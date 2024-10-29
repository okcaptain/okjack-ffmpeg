#include "avs3_cnst_com.h"
#include "avs3_rom_com.h"
#include "avs3_prot_dec.h"
#include "avs3_stat_dec.h"

#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "decode.h"
#include "codec_internal.h"

typedef struct AV3AContext {
    AVClass* class;
    int inited;
    uint16_t nn_type;
    uint16_t coding_profile;
    uint16_t sample_rate_index;
    uint16_t crc_result;
    uint16_t channel_number_index;
    uint16_t sound_bed_type;
    uint16_t objs_num;
    uint16_t bitrate_index_per_channel;
    uint16_t hoa_order;
    uint16_t resolution;
    uint16_t bitrate_index;
    uint16_t frame_bytes;
    int32_t header_bytes;
    int32_t bitrate_per_obj;
    size_t size;
    int16_t *data;
    FILE *model_fp;
    AVCodecContext *avctx;
    AVS3DecoderHandle handle;
} AV3AContext;

typedef struct AV3AChannelLayout {
    int32_t nb_channels;
    uint64_t mask;
} AV3AChannelLayout;

//#define AV3A_CH_TOP_SIDE_LEFT          (1ULL << 36 )
//#define AV3A_CH_TOP_SIDE_RIGHT         (1ULL << 37 )
//#define AV3A_CH_BOTTOM_FRONT_CENTER    (1ULL << 38 )
//#define AV3A_CH_BOTTOM_FRONT_LEFT      (1ULL << 39 )
//#define AV3A_CH_BOTTOM_FRONT_RIGHT     (1ULL << 40 )


//#define AV3A_CH_LAYOUT_22_2  (AV_CH_LAYOUT_5POINT1_BACK|AV_CH_FRONT_LEFT_OF_CENTER|AV_CH_FRONT_RIGHT_OF_CENTER|AV_CH_BACK_CENTER|AV_CH_LOW_FREQUENCY_2|AV_CH_SIDE_LEFT|AV_CH_SIDE_RIGHT|AV_CH_TOP_FRONT_LEFT|\
//		              AV_CH_TOP_FRONT_RIGHT|AV_CH_TOP_FRONT_CENTER|AV_CH_TOP_CENTER|AV_CH_TOP_BACK_LEFT|AV_CH_TOP_BACK_RIGHT|AV3A_CH_TOP_SIDE_LEFT|AV3A_CH_TOP_SIDE_RIGHT|AV_CH_TOP_BACK_CENTER|\
//			      AV3A_CH_BOTTOM_FRONT_CENTER|AV3A_CH_BOTTOM_FRONT_LEFT|AV3A_CH_BOTTOM_FRONT_RIGHT)
//#define AV3A_CH_LAYOUT_5_1_2   (AV_CH_LAYOUT_5POINT1|AV_CH_TOP_FRONT_CENTER|AV_CH_TOP_BACK_CENTER)
//#define AV3A_CH_LAYOUT_5_1_4   (AV_CH_LAYOUT_5POINT1|AV_CH_TOP_FRONT_LEFT|AV_CH_TOP_FRONT_RIGHT|AV_CH_TOP_BACK_LEFT|AV_CH_TOP_BACK_RIGHT)
//#define AV3A_CH_LAYOUT_7_1_2   (AV_CH_LAYOUT_7POINT1|AV_CH_TOP_FRONT_CENTER|AV_CH_TOP_BACK_CENTER)
//#define AV3A_CH_LAYOUT_7_1_4   (AV_CH_LAYOUT_7POINT1|AV_CH_TOP_FRONT_LEFT|AV_CH_TOP_FRONT_RIGHT|AV_CH_TOP_BACK_LEFT|AV_CH_TOP_BACK_RIGHT)
//#define AV3A_CH_LAYOUT_8POINT1 (AV_CH_LAYOUT_OCTAGONAL|AV_CH_TOP_CENTER)

#define AV3A_CHANNEL_LAYOUT_MASK(nb, m) \
    { .nb_channels = (nb), .mask = (m) }

static const AV3AChannelLayout av3a_channel_layout[] = {
        AV3A_CHANNEL_LAYOUT_MASK(1, AV_CH_LAYOUT_MONO),
        AV3A_CHANNEL_LAYOUT_MASK(2, AV_CH_LAYOUT_STEREO),
        AV3A_CHANNEL_LAYOUT_MASK(6, AV_CH_LAYOUT_5POINT1),
        AV3A_CHANNEL_LAYOUT_MASK(8, AV_CH_LAYOUT_7POINT1),
        AV3A_CHANNEL_LAYOUT_MASK(12, AV_CH_LAYOUT_10POINT2),  // CHANNEL_CONFIG_MC_10_2
        AV3A_CHANNEL_LAYOUT_MASK(24, AV_CH_LAYOUT_22POINT2),
        AV3A_CHANNEL_LAYOUT_MASK(4,  AV_CH_LAYOUT_4POINT0),
        AV3A_CHANNEL_LAYOUT_MASK(8,  AV_CH_LAYOUT_5_1_2),
        AV3A_CHANNEL_LAYOUT_MASK(10, AV_CH_LAYOUT_5_1_4),
        AV3A_CHANNEL_LAYOUT_MASK(10, AV_CH_LAYOUT_7_1_2),
        AV3A_CHANNEL_LAYOUT_MASK(12, AV_CH_LAYOUT_7_1_4),
        AV3A_CHANNEL_LAYOUT_MASK(4,  AV_CH_LAYOUT_4POINT0),
        AV3A_CHANNEL_LAYOUT_MASK(9,  AV_CH_LAYOUT_9POINT0),  // CHANNEL_CONFIG_HOA_ORDER2
        AV3A_CHANNEL_LAYOUT_MASK(16, AV_CH_LAYOUT_HEXADECAGONAL), // CHANNEL_CONFIG_HOA_ORDER3
        {0},
};

static int16_t av3a_get_mc_channels(const ChannelNumConfig num_config)
{
    int16_t channels = 0;

    for (int i = 0; i < AVS3_SIZE_MC_CONFIG_TABLE; ++i) {
        if (mcChannelConfigTable[i].channelNumConfig == num_config) {
            channels= mcChannelConfigTable[i].numChannels;
            break;
        }
    }

    return channels;
}

static int av3a_parse_header(uint8_t *buf, AV3AContext *header)
{
    uint32_t bit_pos = 0;

    // sync word, 12bits
    if (GetNextIndice(buf, &bit_pos, NBITS_SYNC_WORD) != SYNC_WORD_COMPAT)
        return AVERROR(EINVAL);

    // codec id, 4bits
    if (GetNextIndice(buf, &bit_pos, NBITS_AUDIO_CODEC_ID) != 2)
        return AVERROR(EINVAL);

    // anc data index, fixed to 0 in HW branch, 1 bit
    if (GetNextIndice(buf, &bit_pos, NBITS_ANC_DATA_INDEX) == 1)
        return AVERROR(EINVAL);

    header->nn_type = GetNextIndice(buf, &bit_pos, NBITS_NN_TYPE);
    header->coding_profile = GetNextIndice(buf, &bit_pos, NBITS_CODING_PROFILE);
    header->sample_rate_index = GetNextIndice(buf, &bit_pos, NBITS_SAMPLING_RATE_INDEX);

    header->crc_result = GetNextIndice(buf, &bit_pos, AVS3_BS_BYTE_SIZE);
    header->crc_result = header->crc_result << AVS3_BS_BYTE_SIZE;

    if (header->coding_profile == 0) {
        header->channel_number_index = GetNextIndice(buf, &bit_pos, NBITS_CHANNEL_NUMBER_INDEX);
    } else if (header->coding_profile == 1) {
        header->sound_bed_type = GetNextIndice(buf, &bit_pos, NBITS_SOUNDBED_TYPE);
        if (header->sound_bed_type == 0) {
            header->objs_num = GetNextIndice(buf, &bit_pos, NBITS_NUM_OBJS);
            header->objs_num += 1;
            header->bitrate_index_per_channel = GetNextIndice(buf, &bit_pos, NBITS_BITRATE_INDEX);
        }
        else if (header->sound_bed_type == 1) {
            header->channel_number_index = GetNextIndice(buf, &bit_pos, NBITS_CHANNEL_NUMBER_INDEX);
            header->bitrate_index = GetNextIndice(buf, &bit_pos, NBITS_BITRATE_INDEX);
            header->objs_num = GetNextIndice(buf, &bit_pos, NBITS_NUM_OBJS);
            header->objs_num += 1;
            header->bitrate_index_per_channel = GetNextIndice(buf, &bit_pos, NBITS_BITRATE_INDEX);
        }
    } else if (header->coding_profile == 2) {
        header->hoa_order = GetNextIndice(buf, &bit_pos, NBITS_HOA_ORDER);
        header->hoa_order += 1;
        if (header->hoa_order == 1)
            header->channel_number_index = CHANNEL_CONFIG_HOA_ORDER1;
        else if (header->hoa_order == 2)
            header->channel_number_index = CHANNEL_CONFIG_HOA_ORDER2;
        else if (header->hoa_order == 3)
            header->channel_number_index = CHANNEL_CONFIG_HOA_ORDER3;
        else
            header->channel_number_index = CHANNEL_CONFIG_UNKNOWN;
    }

    header->resolution = GetNextIndice(buf, &bit_pos, NBITS_RESOLUTION);
    if (header->coding_profile != 1) {
        header->bitrate_index = GetNextIndice(buf, &bit_pos, NBITS_BITRATE_INDEX);
    }

    header->crc_result += GetNextIndice(buf, &bit_pos, AVS3_BS_BYTE_SIZE);
    header->header_bytes = (int32_t)(ceil((float)bit_pos / 8));

    return 0;
}

static int av3a_update_params(AV3AContext *header, AVCodecContext *avctx)
{
    header->handle->outputFs = avs3SamplingRateTable[header->sample_rate_index];
    header->handle->frameLength = FRAME_LEN;

    if (header->resolution <= 2)
        header->handle->bitDepth = (header->resolution + 1) << 3;

    header->handle->nnTypeConfig = (NnTypeConfig)header->nn_type;

    if (header->coding_profile == 0) {
        header->handle->isMixedContent = 0;
        header->handle->channelNumConfig = (ChannelNumConfig)header->channel_number_index;
        if (header->handle->channelNumConfig == CHANNEL_CONFIG_MONO) {
            header->handle->avs3CodecFormat = AVS3_MONO_FORMAT;
            header->handle->numChansOutput = 1;
        } else if (header->handle->channelNumConfig == CHANNEL_CONFIG_STEREO) {
            header->handle->avs3CodecFormat = AVS3_STEREO_FORMAT;
            header->handle->numChansOutput = 2;
        } else if (header->handle->channelNumConfig <= CHANNEL_CONFIG_MC_7_1_4) {
            header->handle->avs3CodecFormat = AVS3_MC_FORMAT;
            header->handle->numChansOutput = av3a_get_mc_channels(header->handle->channelNumConfig);
            if (header->handle->channelNumConfig == CHANNEL_CONFIG_MC_4_0)
                header->handle->hasLfe = 0;
            else
                header->handle->hasLfe = 1;
        } else
            return AVERROR(EINVAL);
    } else if (header->coding_profile == 1) {
        header->handle->isMixedContent = 1;
        header->handle->soundBedType = header->sound_bed_type;
        if (header->sound_bed_type == 0) {
            header->handle->numObjsOutput = header->objs_num;
            header->handle->numChansOutput = header->objs_num;
            if (header->objs_num == 1)
                header->handle->avs3CodecFormat = AVS3_MONO_FORMAT;
            else if (header->objs_num == 2)
                header->handle->avs3CodecFormat = AVS3_STEREO_FORMAT;
            else if (header->objs_num == 3)
                header->handle->avs3CodecFormat = AVS3_MC_FORMAT;
            header->handle->channelNumConfig = CHANNEL_CONFIG_UNKNOWN;
            header->bitrate_per_obj = codecBitrateConfigTable[CHANNEL_CONFIG_MONO].bitrateTable[header->bitrate_index];
            header->handle->totalBitrate = header->objs_num * header->bitrate_per_obj;
            header->handle->hasLfe = 0;
        } else if (header->sound_bed_type == 1) {
            header->handle->avs3CodecFormat = AVS3_MC_FORMAT;
            header->handle->channelNumConfig = (ChannelNumConfig)header->channel_number_index;
            header->handle->bitrateBedMc= codecBitrateConfigTable[header->handle->channelNumConfig].bitrateTable[header->bitrate_index];
            header->handle->numChansOutput = av3a_get_mc_channels(header->handle->channelNumConfig);
            header->handle->numObjsOutput = header->objs_num;
            header->handle->bitratePerObj = codecBitrateConfigTable[CHANNEL_CONFIG_MONO].bitrateTable[header->bitrate_index_per_channel];
            header->handle->numChansOutput += header->handle->numObjsOutput;
            header->handle->totalBitrate = header->handle->bitrateBedMc + header->handle->numObjsOutput * header->handle->bitratePerObj;
            if (header->handle->channelNumConfig == CHANNEL_CONFIG_STEREO || header->handle->channelNumConfig == CHANNEL_CONFIG_MC_4_0)
                header->handle->hasLfe = 0;
            else
                header->handle->hasLfe = 1;
        }
    } else if (header->coding_profile == 2) {
        header->handle->isMixedContent = 0;
        header->handle->avs3CodecFormat = AVS3_HOA_FORMAT;
        header->handle->channelNumConfig = (ChannelNumConfig)header->channel_number_index;
        header->handle->numChansOutput = (header->hoa_order + 1) * (header->hoa_order + 1);
    }

    if (header->handle->numChansOutput <= 0)
        return AVERROR(EINVAL);

    if (header->handle->isMixedContent == 0) {
        header->handle->totalBitrate = codecBitrateConfigTable[header->handle->channelNumConfig].bitrateTable[header->bitrate_index];
    }

    avctx->sample_rate = header->handle->outputFs;
    avctx->bit_rate = header->handle->totalBitrate;
    avctx->bits_per_raw_sample = header->handle->bitDepth;
    // avctx->channel_layout = av3a_channel_layout[header->channel_number_index].mask;
    // avctx->channels = av3a_channel_layout[header->channel_number_index].nb_channels;
    avctx->channels = header->handle->numChansOutput;
    avctx->channel_layout = av_get_default_channel_layout(avctx->channels);
    if (avctx->channel_layout <= 0) {
        avctx->channel_layout = 0;
        for (int32_t c = 0; c < avctx->channels; ++c)
            avctx->channel_layout |= 1 << c;
    }

    avctx->frame_size = FRAME_LEN;
    if (avctx->bits_per_raw_sample == 8)
        avctx->sample_fmt = AV_SAMPLE_FMT_U8;
    else if (avctx->bits_per_raw_sample == 16)
        avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    else if (avctx->bits_per_raw_sample == 24)
        avctx->sample_fmt = AV_SAMPLE_FMT_S32;

    header->handle->bitsPerFrame = (int32_t)(((float)header->handle->totalBitrate / (float)header->handle->outputFs) * FRAME_LEN);
    if (header->handle->isMixedContent == 0) {
        if (header->handle->avs3CodecFormat == AVS3_MONO_FORMAT)
            header->handle->bitsPerFrame -= NBITS_FRAME_HEADER_MONO;
        else if (header->handle->avs3CodecFormat == AVS3_STEREO_FORMAT)
            header->handle->bitsPerFrame -= NBITS_FRAME_HEADER_STEREO;
        else if (header->handle->avs3CodecFormat == AVS3_MC_FORMAT)
            header->handle->bitsPerFrame -= NBITS_FRAME_HEADER_MC;
        else if (header->handle->avs3CodecFormat == AVS3_HOA_FORMAT)
            header->handle->bitsPerFrame -= NBITS_FRAME_HEADER_HOA;
    } else {
        if (header->sound_bed_type == 0)
            header->handle->bitsPerFrame -= NBITS_FRAME_HEADER_MIX_SBT0;
        else if (header->sound_bed_type == 1)
            header->handle->bitsPerFrame -= NBITS_FRAME_HEADER_MIX_SBT1;
    }

    header->frame_bytes= (uint32_t)(ceil((float)header->handle->bitsPerFrame / 8));

    return 0;
}

static av_cold int av3a_decode_init(AVCodecContext *avctx)
{
    AV3AContext *s = avctx->priv_data;
    s->avctx = avctx;

    if (!(s->handle = Avs3AllocDecoder()))
        return AVERROR(ENOMEM);

    s->size = 0;
    s->data = NULL;
    s->handle->avs3CodecCore = AVS3_MDCT_CORE;
    s->inited = 0;
    s->model_fp = NULL;

    return 0;
}

static av_cold int av3a_decode_close(AVCodecContext *avctx)
{
    AV3AContext *s = avctx->priv_data;

    if (s->data != NULL) {
        av_free(s->data);
        s->data = NULL;
        s->size = 0;
    }

    if (s->handle) {
        Avs3DecoderDestroy(s->handle, &s->model_fp);
        s->handle = NULL;
    }
    s->inited = 0;

    return 0;
}

static int av3a_decode_frame(AVCodecContext *avctx, void *data,
                             int *got_frame_ptr, AVPacket *avpkt)
{
    AV3AContext *s = avctx->priv_data;
    int ret;
    AVFrame *frame = (AVFrame*)data;

    if (frame) {
        if ((ret = av3a_parse_header(avpkt->data, s)) < 0)
            return ret;

        if ((ret = av3a_update_params(s, avctx)) < 0)
            return ret;

        if (s->inited == 0) {
            Avs3InitDecoder(s->handle, &s->model_fp);
            s->inited = 1;
        }

        memcpy(s->handle->hBitstream, avpkt->data + s->header_bytes, s->frame_bytes);

        if (!s->data) {
            s->size = (size_t)avctx->channels * (size_t)avctx->frame_size * sizeof(int16_t);
            if (!(s->data = (int16_t*)av_mallocz(s->size)))
            return AVERROR(ENOMEM);
        }
        Avs3Decode(s->handle, s->data);

        if (s->handle->hMetadataDec && (s->handle->hMetadataDec->avs3MetaData.hasStaticMeta || s->handle->hMetadataDec->avs3MetaData.hasDynamicMeta)) {
            AVFrameSideData * sd = av_frame_new_side_data(frame, AV_FRAME_DATA_AUDIO_VIVID_METADATA, (int)sizeof(Avs3MetaData));
            if (!sd)
                return AVERROR(ENOMEM);
            memcpy(sd->data, &s->handle->hMetadataDec->avs3MetaData, sizeof(Avs3MetaData));
        }

        ResetBitstream(s->handle->hBitstream);

        frame->nb_samples = avctx->frame_size;
        frame->sample_rate = avctx->sample_rate;
        frame->channels = avctx->channels;

        if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
            return ret;

        memcpy(frame->data[0], s->data, frame->linesize[0]);
        memset(s->data, 0, s->size);

        *got_frame_ptr = 1;
        return s->header_bytes + s->frame_bytes;
    }

    *got_frame_ptr = 0;

    return 0;
}

static void flush(AVCodecContext *avctx)
{
}

static const AVOption options[] = {
        {NULL},
};

#ifdef CONFIG_LIBAV3AD
static const AVClass libavs3a_decoder_class = {
	.class_name = "libav3a decoder",
	.item_name = av_default_item_name,
	.option = options,
	.version = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_av3a_decoder = {
	.p.name = "av3a",
     CODEC_LONG_NAME("AV3A (Advanced Audio Coding)"),
	.p.type = AVMEDIA_TYPE_AUDIO,
	.p.id = AV_CODEC_ID_AV3A,
	.priv_data_size = sizeof(AV3AContext),
	.init = av3a_decode_init,
	.close = av3a_decode_close,
    FF_CODEC_DECODE_CB(av3a_decode_frame),
	.p.sample_fmts = (const enum AVSampleFormat[]) {
		AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE
	},
	.p.capabilities = AV_CODEC_CAP_CHANNEL_CONF | AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE |
                          FF_CODEC_CAP_AUTO_THREADS,
	.flush = flush,
	.p.priv_class = &libavs3a_decoder_class,
	.p.wrapper_name = "av3a",
};
#endif
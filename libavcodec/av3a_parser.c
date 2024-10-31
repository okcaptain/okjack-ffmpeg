#include "parser.h"

#include "avs3_cnst_com.h"
#include "avs3_rom_com.h"
#include "avs3_prot_dec.h"
#include "avs3_stat_dec.h"

typedef struct AV3AParseContext {
    ParseContext pc;

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

    int channels;
    int sample_rate;
    int bit_rate;
    int bits_per_raw_sample;
    int samples;
    uint64_t channel_layout;
    int frame_size;
    int sample_fmt;
    int service_type;

    int remaining_size;
    uint64_t state;

    int need_next_header;
    enum AVCodecID codec_id;
} AV3AParseContext;

typedef struct AV3AChannelLayout {
    int32_t nb_channels;
    uint64_t mask;
} AV3AChannelLayout;

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

static int av3a_parse_header(uint8_t *buf, AV3AParseContext *header)
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

static int av3a_update_params(AV3AParseContext *header)
{
    int outputFs = avs3SamplingRateTable[header->sample_rate_index];
    int frameLength = FRAME_LEN;
    int bitDepth;
    NnTypeConfig nnTypeConfig;
    int isMixedContent;
    ChannelNumConfig channelNumConfig;
    int avs3CodecFormat;
    int numChansOutput;
    int hasLfe;
    uint16_t soundBedType;
    uint16_t numObjsOutput;
    int totalBitrate;
    int bitrateBedMc;
    int bitratePerObj;
    int32_t bitsPerFrame;

    if (header->resolution <= 2)
        bitDepth = (header->resolution + 1) << 3;

    nnTypeConfig = (NnTypeConfig)header->nn_type;

    if (header->coding_profile == 0) {
        isMixedContent = 0;
        channelNumConfig = (ChannelNumConfig)header->channel_number_index;
        if (channelNumConfig == CHANNEL_CONFIG_MONO) {
            avs3CodecFormat = AVS3_MONO_FORMAT;
            numChansOutput = 1;
        } else if (channelNumConfig == CHANNEL_CONFIG_STEREO) {
            avs3CodecFormat = AVS3_STEREO_FORMAT;
            numChansOutput = 2;
        } else if (channelNumConfig <= CHANNEL_CONFIG_MC_7_1_4) {
            avs3CodecFormat = AVS3_MC_FORMAT;
            numChansOutput = av3a_get_mc_channels(channelNumConfig);
            if (channelNumConfig == CHANNEL_CONFIG_MC_4_0)
                hasLfe = 0;
            else
                hasLfe = 1;
        } else
            return AVERROR(EINVAL);
    } else if (header->coding_profile == 1) {
        isMixedContent = 1;
        soundBedType = header->sound_bed_type;
        if (header->sound_bed_type == 0) {
            numObjsOutput = header->objs_num;
            numChansOutput = header->objs_num;
            if (header->objs_num == 1)
                avs3CodecFormat = AVS3_MONO_FORMAT;
            else if (header->objs_num == 2)
                avs3CodecFormat = AVS3_STEREO_FORMAT;
            else if (header->objs_num == 3)
                avs3CodecFormat = AVS3_MC_FORMAT;
            channelNumConfig = CHANNEL_CONFIG_UNKNOWN;
            header->bitrate_per_obj = codecBitrateConfigTable[CHANNEL_CONFIG_MONO].bitrateTable[header->bitrate_index];
            totalBitrate = header->objs_num * header->bitrate_per_obj;
            hasLfe = 0;
        } else if (header->sound_bed_type == 1) {
            avs3CodecFormat = AVS3_MC_FORMAT;
            channelNumConfig = (ChannelNumConfig)header->channel_number_index;
            bitrateBedMc = codecBitrateConfigTable[channelNumConfig].bitrateTable[header->bitrate_index];
            numChansOutput = av3a_get_mc_channels(channelNumConfig);
            numObjsOutput = header->objs_num;
            bitratePerObj = codecBitrateConfigTable[CHANNEL_CONFIG_MONO].bitrateTable[header->bitrate_index_per_channel];
            numChansOutput += numObjsOutput;
            totalBitrate = bitrateBedMc + numObjsOutput * bitratePerObj;
            if (channelNumConfig == CHANNEL_CONFIG_STEREO || channelNumConfig == CHANNEL_CONFIG_MC_4_0)
                hasLfe = 0;
            else
                hasLfe = 1;
        }
    } else if (header->coding_profile == 2) {
        isMixedContent = 0;
        avs3CodecFormat = AVS3_HOA_FORMAT;
        channelNumConfig = (ChannelNumConfig)header->channel_number_index;
        numChansOutput = (header->hoa_order + 1) * (header->hoa_order + 1);
    }

    if (numChansOutput <= 0)
        return AVERROR(EINVAL);

    if (isMixedContent == 0) {
        totalBitrate = codecBitrateConfigTable[channelNumConfig].bitrateTable[header->bitrate_index];
    }

    header->sample_rate = outputFs;
    header->bit_rate = totalBitrate;
    header->bits_per_raw_sample = bitDepth;
    // header->channel_layout = av3a_channel_layout[header->channel_number_index].mask;
    // header->channels = av3a_channel_layout[header->channel_number_index].nb_channels;
    header->channels = numChansOutput;
    header->channel_layout = av_get_default_channel_layout(header->channels);
    if (header->channel_layout <= 0)
    {
        header->channel_layout = 0;
        for (int32_t c = 0; c < header->channels; ++c)
            header->channel_layout |= 1 << c;
    }

    header->frame_size = FRAME_LEN;
    if (header->bits_per_raw_sample == 8)
        header->sample_fmt = AV_SAMPLE_FMT_U8;
    else if (header->bits_per_raw_sample == 16)
        header->sample_fmt = AV_SAMPLE_FMT_S16;
    else if (header->bits_per_raw_sample == 24)
        header->sample_fmt = AV_SAMPLE_FMT_S32;

    bitsPerFrame = (int32_t)(((float)totalBitrate / (float)outputFs) * FRAME_LEN);
    if (isMixedContent == 0) {
        if (avs3CodecFormat == AVS3_MONO_FORMAT)
            bitsPerFrame -= NBITS_FRAME_HEADER_MONO;
        else if (avs3CodecFormat == AVS3_STEREO_FORMAT)
            bitsPerFrame -= NBITS_FRAME_HEADER_STEREO;
        else if (avs3CodecFormat == AVS3_MC_FORMAT)
            bitsPerFrame -= NBITS_FRAME_HEADER_MC;
        else if (avs3CodecFormat == AVS3_HOA_FORMAT)
            bitsPerFrame -= NBITS_FRAME_HEADER_HOA;
    } else {
        if (header->sound_bed_type == 0)
            bitsPerFrame -= NBITS_FRAME_HEADER_MIX_SBT0;
        else if (header->sound_bed_type == 1)
            bitsPerFrame -= NBITS_FRAME_HEADER_MIX_SBT1;
    }

    header->frame_bytes = (uint32_t)(ceil((float)bitsPerFrame / 8));

    return 0;
}

static int ff_av3a_parse(AVCodecParserContext *s1,
                         AVCodecContext *avctx,
                         const uint8_t **poutbuf, int *poutbuf_size,
                         const uint8_t *buf, int buf_size)
{
    AV3AParseContext *s = s1->priv_data;
    // ParseContext *pc = &s->pc;
    int ret;
    *poutbuf = buf;

    if ((ret = av3a_parse_header(buf, s)) < 0) {
        *poutbuf_size = buf_size;
        return ret;
    }

    if ((ret = av3a_update_params(s)) < 0) {
        *poutbuf_size = buf_size;
        return ret;
    }

    if ((s->header_bytes + s->frame_bytes) > buf_size)
        s->frame_bytes = buf_size - s->header_bytes;
    *poutbuf_size = s->header_bytes + s->frame_bytes;

    return s->header_bytes + s->frame_bytes;
}

static av_cold int av3a_parse_init(AVCodecParserContext *s1)
{
    AV3AParseContext *s = s1->priv_data;
    return 0;
}


AVCodecParser ff_av3a_parser = {
        .codec_ids      = { AV_CODEC_ID_AV3A },
        .priv_data_size = sizeof(AV3AParseContext),
        .parser_init    = av3a_parse_init,
        .parser_parse   = ff_av3a_parse,
        .parser_close   = ff_parse_close,
};
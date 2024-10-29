#include "avs3_cnst_com.h"
#include "avs3_prot_com.h"
#include "avs3_rom_com.h"
#include "avs3_prot_dec.h"

#include "avformat.h"
#include "rawdec.h"
#include "internal.h"
#include "libavutil/opt.h"
#include "libavutil/intreadwrite.h"

typedef struct AV3AHeader {
    AVClass *class;
    int16_t codec_fmt;
    uint16_t nn_type;
    uint16_t coding_profile;
    uint16_t sample_rate_index;
    uint16_t crc_result;
    uint16_t channel_number_index;
    uint16_t sound_bed_type;
    uint16_t object_channel_number;
    uint16_t bitrate_index_per_channel;
    uint16_t hoa_order;
    uint16_t resolution;
    uint16_t bitrate_index;
    uint16_t frame_bits;
    uint16_t header_bytes;
    int mixed_content;
    int32_t sample_rate;
    int32_t bitrate_per_obj;
    int32_t total_bitrate;
} AV3AHeader;

static int av3a_parse_header(AVFormatContext *s, int first)
{
    int ret;
    AVPacket pkt;
    uint32_t bit_pos = 0;
    uint8_t bitstream[MAX_NBYTES_FRAME_HEADER];
    int32_t header_bytes;
    AV3AHeader* header = s->priv_data;

    av_init_packet(&pkt);

    ret = av_get_packet(s->pb, &pkt, MAX_NBYTES_FRAME_HEADER);
    if (ret < 0)
        return ret;

    if (ret < MAX_NBYTES_FRAME_HEADER) {
        ret = AVERROR(EIO);
        goto fail;
    }

    ret = AVERROR(EINVAL);
    memcpy(bitstream, pkt.data, MAX_NBYTES_FRAME_HEADER);

    // sync word, 12bits
    if (GetNextIndice(bitstream, &bit_pos, NBITS_SYNC_WORD) != SYNC_WORD_COMPAT)
        goto fail;

    // codec id, 4bits
    if (GetNextIndice(bitstream, &bit_pos, NBITS_AUDIO_CODEC_ID) != 2)
        goto fail;

    // anc data index, fixed to 0 in HW branch, 1 bit
    if (GetNextIndice(bitstream, &bit_pos, NBITS_ANC_DATA_INDEX) == 1)
        goto fail;

    header->nn_type = GetNextIndice(bitstream, &bit_pos, NBITS_NN_TYPE);
    header->coding_profile = GetNextIndice(bitstream, &bit_pos, NBITS_CODING_PROFILE);
    header->sample_rate_index = GetNextIndice(bitstream, &bit_pos, NBITS_SAMPLING_RATE_INDEX);

    header->crc_result = GetNextIndice(bitstream, &bit_pos, AVS3_BS_BYTE_SIZE);
    header->crc_result = header->crc_result << AVS3_BS_BYTE_SIZE;

    if (header->coding_profile == 0) {
        header->channel_number_index = GetNextIndice(bitstream, &bit_pos, NBITS_CHANNEL_NUMBER_INDEX);
    } else if (header->coding_profile == 1) {
        header->sound_bed_type = GetNextIndice(bitstream, &bit_pos, NBITS_SOUNDBED_TYPE);
        if (header->sound_bed_type == 0) {
            header->object_channel_number = GetNextIndice(bitstream, &bit_pos, NBITS_NUM_OBJS);
            header->object_channel_number += 1;
            header->bitrate_index_per_channel = GetNextIndice(bitstream, &bit_pos, NBITS_BITRATE_INDEX);
        } else if (header->sound_bed_type == 1) {
            header->channel_number_index = GetNextIndice(bitstream, &bit_pos, NBITS_CHANNEL_NUMBER_INDEX);
            header->bitrate_index = GetNextIndice(bitstream, &bit_pos, NBITS_BITRATE_INDEX);
            header->object_channel_number = GetNextIndice(bitstream, &bit_pos, NBITS_NUM_OBJS);
            header->object_channel_number += 1;
            header->bitrate_index_per_channel = GetNextIndice(bitstream, &bit_pos, NBITS_BITRATE_INDEX);
        }
    } else if (header->coding_profile == 2) {
        header->hoa_order = GetNextIndice(bitstream, &bit_pos, NBITS_HOA_ORDER);
        header->hoa_order += 1;
    }

    header->resolution = GetNextIndice(bitstream, &bit_pos, NBITS_RESOLUTION);
    if (header->coding_profile != 1) {
        header->bitrate_index = GetNextIndice(bitstream, &bit_pos, NBITS_BITRATE_INDEX);
    }

    header->crc_result += GetNextIndice(bitstream, &bit_pos, AVS3_BS_BYTE_SIZE);

    header->header_bytes = (int32_t)(ceil((float)bit_pos / 8));

    if (first) {
        avio_seek(s->pb, 0, SEEK_SET);
    } else {
        avio_seek(s->pb, 0 - MAX_NBYTES_FRAME_HEADER, SEEK_CUR);
    }

    if (header->coding_profile == 0) {
        header->mixed_content = 0;
        if (header->channel_number_index == CHANNEL_CONFIG_MONO) {
            header->codec_fmt = AVS3_MONO_FORMAT;
        } else if (header->channel_number_index == CHANNEL_CONFIG_STEREO) {
            header->codec_fmt = AVS3_STEREO_FORMAT;
        } else if (header->channel_number_index <= CHANNEL_CONFIG_MC_7_1_4) {
            header->codec_fmt = AVS3_MC_FORMAT;
        } else {
            goto fail;
        }
    } else if (header->coding_profile == 1) {
        header->mixed_content = 1;
        if (header->sound_bed_type == 0) {
            if (header->object_channel_number == 1)
                header->codec_fmt = AVS3_MONO_FORMAT;
            else if (header->object_channel_number == 2)
                header->codec_fmt = AVS3_STEREO_FORMAT;
            else if (header->object_channel_number == 3)
                header->codec_fmt = AVS3_MC_FORMAT;
            else
                goto fail;
            header->channel_number_index = CHANNEL_CONFIG_UNKNOWN;
            header->bitrate_per_obj = codecBitrateConfigTable[CHANNEL_CONFIG_MONO].bitrateTable[header->bitrate_index_per_channel];
            header->total_bitrate = header->object_channel_number * header->bitrate_per_obj;
        }
    } else if (header->coding_profile == 2) {
        header->mixed_content = 0;
        header->codec_fmt = AVS3_HOA_FORMAT;
        if (header->hoa_order == 1)
            header->channel_number_index = CHANNEL_CONFIG_HOA_ORDER1;
        else if (header->hoa_order == 2)
            header->channel_number_index = CHANNEL_CONFIG_HOA_ORDER2;
        else if (header->hoa_order == 3)
            header->channel_number_index = CHANNEL_CONFIG_HOA_ORDER3;
        else
            goto fail;
    }

    if (header->mixed_content == 0) {
        header->total_bitrate = codecBitrateConfigTable[header->channel_number_index].bitrateTable[header->bitrate_index];
    }
    header->sample_rate = avs3SamplingRateTable[header->sample_rate_index];

    ret = 0;

    fail:
    av_packet_unref(&pkt);
    return ret;
}

static int av3a_probe(const AVProbeData *p)
{
    int score = 0;
    uint32_t bit_pos = 0;
    const uint8_t *ptr = p->buf;

    if (av_match_ext(p->filename, "av3a"))
        score = AVPROBE_SCORE_EXTENSION;

    // 17 = NBITS_SYNC_WORD + NBITS_AUDIO_CODEC_ID + NBITS_ANC_DATA_INDEX
    if (p->buf_size < 3)
        return score;

    // sync word, 12bits
    if (GetNextIndice(ptr, &bit_pos, NBITS_SYNC_WORD) != SYNC_WORD_COMPAT)
        return score;

    // codec id, 4bits
    if (GetNextIndice(ptr, &bit_pos, NBITS_AUDIO_CODEC_ID) != 2)
        return score;

    // anc data index, fixed to 0 in HW branch, 1 bit
    if (GetNextIndice(ptr, &bit_pos, NBITS_ANC_DATA_INDEX) == 1)
        return score;

    return score + AVPROBE_SCORE_RETRY;
}

static int av3a_read_header(AVFormatContext *s)
{
    int ret;

    AV3AHeader* header = (AV3AHeader*)s->priv_data;
    AVStream* st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_AV3A;
    st->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    st->start_time = 0;

    if ((ret = av3a_parse_header(s, 1)) < 0)
        return ret;

    avpriv_set_pts_info(st, 64, 1, header->sample_rate);

    return ret;
}

static int av3a_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret;
    int size;
    uint16_t crc_result;
    AV3AHeader* header = s->priv_data;

    ret = av3a_parse_header(s, 0);
    if (ret < 0)
        return ret;

    header->frame_bits = (int32_t)(((float)header->total_bitrate / (float)header->sample_rate) * 1024);
    if (header->mixed_content == 0) {
        if (header->codec_fmt == AVS3_MONO_FORMAT)
            header->frame_bits -= NBITS_FRAME_HEADER_MONO;
        else if (header->codec_fmt == AVS3_STEREO_FORMAT)
            header->frame_bits -= NBITS_FRAME_HEADER_STEREO;
        else if (header->codec_fmt == AVS3_MC_FORMAT)
            header->frame_bits -= NBITS_FRAME_HEADER_MC;
        else if (header->codec_fmt == AVS3_HOA_FORMAT)
            header->frame_bits -= NBITS_FRAME_HEADER_HOA;
    } else {
        if (header->sound_bed_type == 0)
            header->frame_bits -= NBITS_FRAME_HEADER_MIX_SBT0;
        else if (header->sound_bed_type == 1)
            header->frame_bits -= NBITS_FRAME_HEADER_MIX_SBT1;
    }

    size = (int)(ceil((float)header->frame_bits / 8));
    ret = av_get_packet(s->pb, pkt, size + (int)header->header_bytes);
    if (ret < 0) {
        return ret;
    }

    pkt->pos = avio_tell(s->pb);
    pkt->stream_index = 0;

    crc_result = Crc16(pkt->data + header->header_bytes, pkt->size - header->header_bytes);
    if (crc_result != header->crc_result) {
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }

    return ret;
    fail:
    av_packet_unref(pkt);
    return ret;
}

static const AVOption options[] = {
        { NULL },
};

#ifdef CONFIG_LIBAV3AD
static const AVClass ff_av3a_demuxer_class = {
    .class_name = "av3a",
    .item_name = av_default_item_name,
    .option = options,
    .version = LIBAVUTIL_VERSION_INT,
    .category = AV_CLASS_CATEGORY_DEMUXER,
};

const FFInputFormat ff_av3a_demuxer = {
    .p.name = "av3a",
    .p.long_name = NULL_IF_CONFIG_SMALL("raw av3a"),
    .read_probe = av3a_probe,
    .read_header = av3a_read_header,
    .read_packet = av3a_read_packet,
    .p.flags = AVFMT_GENERIC_INDEX,
    .p.extensions = "av3a",
    .p.mime_type = "audio/av3a",
    .raw_codec_id = AV_CODEC_ID_AV3A,
    .priv_data_size = sizeof(AV3AHeader),
    .p.priv_class = &ff_av3a_demuxer_class,
};
#endif
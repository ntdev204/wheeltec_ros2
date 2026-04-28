


 


  


#include "../dataunnpacker_commondef.h"
#include "../dataunpacker.h"
#include "../dataunnpacker_internal.h"


#include "handler_normalnode.h"

BEGIN_DATAUNPACKER_NS()
	
namespace unpacker{


static _u64 _getSampleDelayOffsetInLegacyMode(const SlamtecLidarTimingDesc& timing)
{

    const _u64 channelBaudRate = timing.native_baudrate? timing.native_baudrate:115200;

    _u64 tranmissionDelay = 1000000ULL * sizeof(rplidar_response_measurement_node_t) * 10 / channelBaudRate;

    if (timing.native_interface_type == LIDARInterfaceType::LIDAR_INTERFACE_ETHERNET)
    {
        tranmissionDelay = 100;
    }


    const _u64 sampleDelay = (timing.sample_duration_uS >> 1);
    const _u64 sampleFilterDelay = timing.sample_duration_uS;

    return sampleFilterDelay + sampleDelay + tranmissionDelay + timing.linkage_delay_uS;
}

UnpackerHandler_NormalNode::UnpackerHandler_NormalNode()
    : _cached_scan_node_buf_pos(0)
{
    _cached_scan_node_buf.resize(sizeof(rplidar_response_measurement_node_t));
    memset(&_cachedTimingDesc, 0, sizeof(_cachedTimingDesc));
;}

UnpackerHandler_NormalNode::~UnpackerHandler_NormalNode()
{

}

_u8 UnpackerHandler_NormalNode::getSampleAnswerType() const
{
	return RPLIDAR_ANS_TYPE_MEASUREMENT;
}

void UnpackerHandler_NormalNode::onData(LIDARSampleDataUnpackerInner* engine, const _u8* data, size_t cnt)
{
    for (size_t pos = 0; pos < cnt; ++pos) {
        _u8 current_data = data[pos];
        switch (_cached_scan_node_buf_pos) {
        case 0:
        {
            _u8 tmp = (current_data >> 1);
            if ((tmp ^ current_data) & 0x1) {

            }
            else {
                continue;
            }

        }
        break;
        case 1:
        {
            if (current_data & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {

            }
            else {
                _cached_scan_node_buf_pos = 0;
                continue;
            }
        }
        break;
        case sizeof(rplidar_response_measurement_node_t) - 1:
        {
            _cached_scan_node_buf[sizeof(rplidar_response_measurement_node_t) - 1] = current_data;
            _cached_scan_node_buf_pos = 0;

            rplidar_response_measurement_node_t* node = reinterpret_cast<rplidar_response_measurement_node_t*>(&_cached_scan_node_buf[0]);
#ifdef _CPU_ENDIAN_BIG
            node->angle_q6_checkbit = le16_to_cpu(node->angle_q6_checkbit);
            node->distance_q2 = le16_to_cpu(node->distance_q2);
#endif

            rplidar_response_measurement_node_hq_t hqNode;
            hqNode.angle_z_q14 = (((node->angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;
            hqNode.dist_mm_q2 = node->distance_q2;
            hqNode.flag = (node->sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
            hqNode.quality = (node->sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            
            
            engine->publishHQNode(engine->getCurrentTimestamp_uS() - _getSampleDelayOffsetInLegacyMode(_cachedTimingDesc), &hqNode);
            continue;

        }
        break;
        }
        _cached_scan_node_buf[_cached_scan_node_buf_pos++] = current_data;
    }
}


void UnpackerHandler_NormalNode::onUnpackerContextSet(LIDARSampleDataUnpacker::UnpackerContextType type, const void* data, size_t size)
{
    if (type == LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING) {
        assert(size == sizeof(_cachedTimingDesc));
        _cachedTimingDesc = *reinterpret_cast<const SlamtecLidarTimingDesc*>(data);
    }
}

void UnpackerHandler_NormalNode::reset()
{
    _cached_scan_node_buf_pos = 0;
}
}


END_DATAUNPACKER_NS()
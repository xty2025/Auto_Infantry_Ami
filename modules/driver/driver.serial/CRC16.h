#include <cstdint>
#include "serial.hpp"

namespace serial {
    void addCRC16(RawSerialWriteData *msg);
    //bool verifyCRC16(SerialReadData::RobotStatus *data);
    bool verifyCRC16(RawSerialData *data);
}
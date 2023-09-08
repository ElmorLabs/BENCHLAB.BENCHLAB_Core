using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BENCHLAB_Core
{
    public static class Helper {

        public static UInt16 CRC16_Calc(byte[] data, int offset, int length) {
            byte x;
            UInt16 crc = 0xFFFF;
            int i = offset;

            if(data.Length >= i + length) {
                while(length-- != 0) {
                    x = (byte)(crc >> 8 ^ data[i++]);
                    x ^= (byte)(x >> 4);
                    crc = (UInt16)((crc << 8) ^ ((UInt16)(x << 12)) ^ ((UInt16)(x << 5)) ^ ((UInt16)x));
                }
            }

            return crc;
        }
    }
}

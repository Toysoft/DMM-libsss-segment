/*
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>

#define SYNC_BYTE 0x47

extern void segmentOffset(uint8_t *buffer, uint32_t length, int16_t offset);

struct HiLoExt {
    uint8_t hi;
    uint32_t lo;
    uint16_t ext;
};

struct HiLoExt readPcr(uint8_t *buffer) {
    struct HiLoExt result;

    result.hi = *(buffer) >> 7 & 0x1;

    result.lo = ((uint8_t)*(buffer) << 24 & 0xFF000000) | ((uint8_t)*(buffer + 1) << 16 & 0xFF0000) | ((uint8_t)*(buffer + 2) << 8 & 0xFF00) | (uint8_t)*(buffer + 3);
    result.lo = (result.lo << 1 &0xFFFFFFFE) | ((uint8_t)*(buffer + 4) >> 7 & 0x1);

    result.ext = ((uint8_t)*(buffer + 4) << 8 & 0x100) | (uint8_t)*(buffer + 5);

    return result;
}

void writePcr(uint8_t *buffer, struct HiLoExt pcr) {
    *(buffer    ) = ((uint8_t)(pcr.hi <<  7 & 0x80)) | (pcr.lo >> 25 & 0x7F);
    *(buffer + 1) =  (uint8_t)(pcr.lo >> 17 & 0xFF);
    *(buffer + 2) =  (uint8_t)(pcr.lo >>  9 & 0xFF);
    *(buffer + 3) =  (uint8_t)(pcr.lo >>  1 & 0xFF);
    *(buffer + 4) = ((uint8_t)(pcr.lo <<  7 & 0x80)) | (*(buffer + 4) & 0x7E) | (pcr.ext >> 8 & 0x1);
    *(buffer + 5) =  (uint8_t)(pcr.ext      & 0xFF);
}

struct HiLoExt readPtsDts(uint8_t *buffer) {
    struct HiLoExt result;

    result.hi = (uint8_t)*(buffer    ) >>  3 & 0x1;

    result.lo = ((uint8_t)*(buffer    ) << 29 & 0xC0000000) | ((uint8_t)*(buffer + 1) << 22 & 0x3F000000)
              | ((uint8_t)*(buffer + 1) << 22 & 0xC00000  ) | ((uint8_t)*(buffer + 2) << 14 & 0x3F0000  )
              | ((uint8_t)*(buffer + 2) << 14 & 0x8000    ) | ((uint8_t)*(buffer + 3) <<  7 & 0x7F00    )
              | ((uint8_t)*(buffer + 3) <<  7 & 0x80      ) | ((uint8_t)*(buffer + 4) >>  1 & 0x7F      );

    result.ext = 0;

    return result;
}

void writePtsDts(uint8_t *buffer, struct HiLoExt pts_dts) {
    *(buffer    ) = ((uint8_t)*(buffer) & 0xF1) | (pts_dts.hi << 3 & 0x8) | (uint8_t)(pts_dts.lo >> 29 & 0x6);
    *(buffer + 1) = (uint8_t)(pts_dts.lo >> 22 & 0xFF);
    *(buffer + 2) = (uint8_t)(pts_dts.lo >> 14 & 0xFE) | ((uint8_t)*(buffer + 2) & 0x1);
    *(buffer + 3) = (uint8_t)(pts_dts.lo >>  7 & 0xFF);
    *(buffer + 4) = (uint8_t)(pts_dts.lo <<  1 & 0xFE) | ((uint8_t)*(buffer + 4) & 0x1);
}

struct HiLoExt *addHiLoExt(struct HiLoExt *hiLo, int32_t value) {
    if (value > 0) {
        (*hiLo).lo += value;
        (*hiLo).hi += ((*hiLo).lo < value);
    } else if (value < 0) {
        value = labs(value);
        (*hiLo).hi -= ((*hiLo).lo < value);
        (*hiLo).lo -= value;
    }

    return hiLo;
}

void segmentOffset(uint8_t *buffer, uint32_t length, int16_t offset) {
    struct HiLoExt pcr, pts, dts;
    uint8_t afc, ptsDtsIndicator, opcrOffset;
    int32_t pesStartCode, timeOffset, pos = 0;

    timeOffset = offset * 187200;

    while ((pos < length) && ((uint8_t) *(buffer+pos) == SYNC_BYTE))
        {
            afc = *(buffer+pos+3) >> 4 & 0x3; // 1 – no adaptation field, payload only, 2 – adaptation field only, no payload, 3 – adaptation field followed by payload
            if (((afc == 0x2) || (afc == 0x3)) && ((uint8_t)*(buffer + pos + 4) > 0)) {
                opcrOffset = 0;

                if (((uint8_t)*(buffer + pos + 5) >> 4 & 0x1) == 1) { // PCR flag
                    pcr = readPcr(buffer + pos + 6);
                    addHiLoExt(&pcr, timeOffset);
                    writePcr(buffer + pos + 6, pcr);

                    opcrOffset += 6;
                }

                if (((uint8_t)*(buffer + pos + 5) >> 3 & 0x1) == 1) { // OPCR flag
                    pcr = readPcr(buffer + pos + 6 + opcrOffset);
                    addHiLoExt(&pcr, timeOffset);
                    writePcr(buffer + pos + 6 + opcrOffset, pcr);
                }

            }

            char payloadOffset = 0;
            if (afc == 0x1) {
                payloadOffset = 4;
            }
            else if (afc == 0x3) {
                payloadOffset = 5 + (uint8_t)*(buffer + pos + 4);
            }

            if (payloadOffset > 0) {
                pesStartCode = ((uint8_t)*(buffer + pos + payloadOffset    ) << 24 & 0xFF000000)
                             | ((uint8_t)*(buffer + pos + payloadOffset + 1) << 16 & 0xFF0000  )
                             | ((uint8_t)*(buffer + pos + payloadOffset + 2) <<  8 & 0xFF00    )
                             | ((uint8_t)*(buffer + pos + payloadOffset + 3)                   );
                if ((pesStartCode >= 0x1C0) && (pesStartCode <= 0x1EF)) // pes for video or audio stream
                {
                    if (((uint8_t)*(buffer + pos + payloadOffset + 6) >> 6 & 0x3) == 0x2) // optional pes header marker bits
                    {
                        ptsDtsIndicator = (uint8_t)*(buffer + pos + payloadOffset + 7) >> 6 &0x3; // 3 = both present, 1 is forbidden, 2 = only PTS, 0 no PTS or DTS

                        if (ptsDtsIndicator > 1)
                        {
                            pts = readPtsDts(buffer + pos + payloadOffset + 9);
                            addHiLoExt(&pts, timeOffset);
                            writePtsDts(buffer + pos + payloadOffset + 9, pts);

                            if (ptsDtsIndicator == 3) {
                                dts = readPtsDts(buffer + pos + payloadOffset + 14);
                                addHiLoExt(&dts, timeOffset);
                                writePtsDts(buffer + pos + payloadOffset + 14, dts);
                            }
                        }
                    }
                }
            }

            pos += 188;
        }
}

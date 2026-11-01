#include "application/services/CryptoUtils.h"
#include <cstring>

namespace gps::app {

namespace {

constexpr uint32_t MD5_K[64] = {
    0xd76aa478U, 0xe8c7b756U, 0x242070dbU, 0xc1bdceeeU, 0xf57c0fafU, 0x4787c62aU, 0xa8304613U, 0xfd469501U,
    0x698098d8U, 0x8b44f7afU, 0xffff5bb1U, 0x895cd7beU, 0x6b901122U, 0xfd987193U, 0xa679438eU, 0x49b40821U,
    0xf61e2562U, 0xc040b340U, 0x265e5a51U, 0xe9b6c7aaU, 0xd62f105dU, 0x02441453U, 0xd8a1e681U, 0xe7d3fbc8U,
    0x21e1cde6U, 0xc33707d6U, 0xf4d50d87U, 0x455a14edU, 0xa9e3e905U, 0xfcefa3f8U, 0x676f02d9U, 0x8d2a4c8aU,
    0xfffa3942U, 0x8771f681U, 0x6d9d6122U, 0xfde5380cU, 0xa4beea44U, 0x4bdecfa9U, 0xf6bb4b60U, 0xbebfbc70U,
    0x289b7ec6U, 0xeaa127faU, 0xd4ef3085U, 0x04881d05U, 0xd9d4d039U, 0xe6db99e5U, 0x1fa27cf8U, 0xc4ac5665U,
    0xf4292244U, 0x432aff97U, 0xab9423a7U, 0xfc93a039U, 0x655b59c3U, 0x8f0ccc92U, 0xffeff47dU, 0x85845dd1U,
    0x6fa87e4fU, 0xfe2ce6e0U, 0xa3014314U, 0x4e0811a1U, 0xf7537e82U, 0xbd3af235U, 0x2ad7d2bbU, 0xeb86d391U
};

constexpr uint8_t MD5_S[64] = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21
};

inline uint32_t rotl32(uint32_t x, uint8_t n) {
    return (x << n) | (x >> (32 - n));
}

constexpr uint8_t SBOX[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

constexpr uint8_t INV_SBOX[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
};

constexpr uint8_t RCON[11] = {
    0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};

inline uint8_t xtime(uint8_t x) {
    return static_cast<uint8_t>((x << 1) ^ (((x >> 7) & 1) * 0x1b));
}

inline uint8_t gmul(uint8_t x, uint8_t y) {
    return static_cast<uint8_t>(
        ((y & 1) * x) ^
        (((y >> 1) & 1) * xtime(x)) ^
        (((y >> 2) & 1) * xtime(xtime(x))) ^
        (((y >> 3) & 1) * xtime(xtime(xtime(x)))));
}

void aesKeyExpansion(const uint8_t key[16], uint8_t roundKeys[176]) {
    std::memcpy(roundKeys, key, 16);
    uint8_t temp[4];
    for (size_t i = 4; i < 44; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            temp[j] = roundKeys[(i - 1) * 4 + j];
        }
        if (i % 4 == 0) {
            const uint8_t k = temp[0];
            temp[0] = SBOX[temp[1]] ^ RCON[i / 4];
            temp[1] = SBOX[temp[2]];
            temp[2] = SBOX[temp[3]];
            temp[3] = SBOX[k];
        }
        for (size_t j = 0; j < 4; ++j) {
            roundKeys[i * 4 + j] = roundKeys[(i - 4) * 4 + j] ^ temp[j];
        }
    }
}

void addRoundKey(uint8_t state[16], const uint8_t* rk) {
    for (size_t i = 0; i < 16; ++i) {
        state[i] ^= rk[i];
    }
}

void aesEncryptBlock(uint8_t state[16], const uint8_t roundKeys[176]) {
    addRoundKey(state, roundKeys);
    for (size_t round = 1; round <= 10; ++round) {
        for (size_t i = 0; i < 16; ++i) state[i] = SBOX[state[i]];
        uint8_t t = state[1]; state[1] = state[5]; state[5] = state[9]; state[9] = state[13]; state[13] = t;
        t = state[2]; state[2] = state[10]; state[10] = t; t = state[6]; state[6] = state[14]; state[14] = t;
        t = state[15]; state[15] = state[11]; state[11] = state[7]; state[7] = state[3]; state[3] = t;
        if (round < 10) {
            for (size_t c = 0; c < 4; ++c) {
                const size_t i = c * 4;
                const uint8_t a0 = state[i], a1 = state[i+1], a2 = state[i+2], a3 = state[i+3];
                const uint8_t tm = a0 ^ a1 ^ a2 ^ a3;
                state[i]   ^= tm ^ xtime(a0 ^ a1);
                state[i+1] ^= tm ^ xtime(a1 ^ a2);
                state[i+2] ^= tm ^ xtime(a2 ^ a3);
                state[i+3] ^= tm ^ xtime(a3 ^ a0);
            }
        }
        addRoundKey(state, roundKeys + round * 16);
    }
}

void aesDecryptBlock(uint8_t state[16], const uint8_t roundKeys[176]) {
    addRoundKey(state, roundKeys + 160);
    for (int round = 9; round >= 0; --round) {
        uint8_t t = state[13]; state[13] = state[9]; state[9] = state[5]; state[5] = state[1]; state[1] = t;
        t = state[2]; state[2] = state[10]; state[10] = t; t = state[6]; state[6] = state[14]; state[14] = t;
        t = state[3]; state[3] = state[7]; state[7] = state[11]; state[11] = state[15]; state[15] = t;
        for (size_t i = 0; i < 16; ++i) state[i] = INV_SBOX[state[i]];
        addRoundKey(state, roundKeys + round * 16);
        if (round > 0) {
            for (size_t c = 0; c < 4; ++c) {
                const size_t i = c * 4;
                const uint8_t a = state[i], b = state[i+1], c_b = state[i+2], d = state[i+3];
                state[i]   = gmul(a, 0x0e) ^ gmul(b, 0x0b) ^ gmul(c_b, 0x0d) ^ gmul(d, 0x09);
                state[i+1] = gmul(a, 0x09) ^ gmul(b, 0x0e) ^ gmul(c_b, 0x0b) ^ gmul(d, 0x0d);
                state[i+2] = gmul(a, 0x0d) ^ gmul(b, 0x09) ^ gmul(c_b, 0x0e) ^ gmul(d, 0x0b);
                state[i+3] = gmul(a, 0x0b) ^ gmul(b, 0x0d) ^ gmul(c_b, 0x09) ^ gmul(d, 0x0e);
            }
        }
    }
}

constexpr const char* B64_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int b64Val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

} // namespace

void CryptoUtils::computeMd5(const uint8_t* data, size_t len, uint8_t outDigest[16]) {
    uint32_t a0 = 0x67452301U;
    uint32_t b0 = 0xefcdab89U;
    uint32_t c0 = 0x98badcfeU;
    uint32_t d0 = 0x10325476U;

    uint8_t msg[512] = {0};
    const size_t safeLen = (len < (sizeof(msg) - 64)) ? len : (sizeof(msg) - 64);
    if (data && safeLen > 0) {
        std::memcpy(msg, data, safeLen);
    }
    msg[safeLen] = 0x80;
    const uint64_t bitsLen = static_cast<uint64_t>(safeLen) * 8ULL;
    const size_t totalBlockLen = ((((safeLen + 8) / 64) + 1) * 64);
    for (size_t i = 0; i < 8; ++i) {
        msg[totalBlockLen - 8 + i] = static_cast<uint8_t>((bitsLen >> (i * 8)) & 0xFFU);
    }

    for (size_t offset = 0; offset < totalBlockLen; offset += 64) {
        uint32_t M[16];
        for (size_t j = 0; j < 16; ++j) {
            const size_t k = offset + j * 4;
            M[j] = static_cast<uint32_t>(msg[k]) |
                   (static_cast<uint32_t>(msg[k + 1]) << 8) |
                   (static_cast<uint32_t>(msg[k + 2]) << 16) |
                   (static_cast<uint32_t>(msg[k + 3]) << 24);
        }

        uint32_t A = a0, B = b0, C = c0, D = d0;
        for (uint32_t i = 0; i < 64; ++i) {
            uint32_t F = 0, g = 0;
            if (i < 16) {
                F = (B & C) | ((~B) & D);
                g = i;
            } else if (i < 32) {
                F = (D & B) | ((~D) & C);
                g = (5 * i + 1) % 16;
            } else if (i < 48) {
                F = B ^ C ^ D;
                g = (3 * i + 5) % 16;
            } else {
                F = C ^ (B | (~D));
                g = (7 * i) % 16;
            }
            F = F + A + MD5_K[i] + M[g];
            A = D;
            D = C;
            C = B;
            B = B + rotl32(F, MD5_S[i]);
        }
        a0 += A; b0 += B; c0 += C; d0 += D;
    }

    const uint32_t words[4] = {a0, b0, c0, d0};
    for (size_t i = 0; i < 4; ++i) {
        outDigest[i * 4 + 0] = static_cast<uint8_t>(words[i] & 0xFFU);
        outDigest[i * 4 + 1] = static_cast<uint8_t>((words[i] >> 8) & 0xFFU);
        outDigest[i * 4 + 2] = static_cast<uint8_t>((words[i] >> 16) & 0xFFU);
        outDigest[i * 4 + 3] = static_cast<uint8_t>((words[i] >> 24) & 0xFFU);
    }
}

void CryptoUtils::deriveLanKey(const char* deviceKey, uint8_t outKey[16]) {
    if (!deviceKey) {
        std::memset(outKey, 0, 16);
        return;
    }
    computeMd5(reinterpret_cast<const uint8_t*>(deviceKey), std::strlen(deviceKey), outKey);
}

size_t CryptoUtils::aes128CbcEncryptPkcs7(const uint8_t key[16],
                                          const uint8_t iv[16],
                                          const uint8_t* plaintext,
                                          size_t plaintextLen,
                                          uint8_t* outCiphertext,
                                          size_t maxOutLen) {
    const size_t padLen = 16 - (plaintextLen % 16);
    const size_t totalLen = plaintextLen + padLen;
    if (!outCiphertext || maxOutLen < totalLen) {
        return 0;
    }

    uint8_t roundKeys[176];
    aesKeyExpansion(key, roundKeys);

    if (plaintext && plaintextLen > 0) {
        std::memcpy(outCiphertext, plaintext, plaintextLen);
    }
    for (size_t i = plaintextLen; i < totalLen; ++i) {
        outCiphertext[i] = static_cast<uint8_t>(padLen);
    }

    uint8_t prevBlock[16];
    std::memcpy(prevBlock, iv, 16);

    for (size_t offset = 0; offset < totalLen; offset += 16) {
        for (size_t i = 0; i < 16; ++i) {
            outCiphertext[offset + i] ^= prevBlock[i];
        }
        aesEncryptBlock(outCiphertext + offset, roundKeys);
        std::memcpy(prevBlock, outCiphertext + offset, 16);
    }

    return totalLen;
}

size_t CryptoUtils::aes128CbcDecryptPkcs7(const uint8_t key[16],
                                          const uint8_t iv[16],
                                          const uint8_t* ciphertext,
                                          size_t ciphertextLen,
                                          uint8_t* outPlaintext,
                                          size_t maxOutLen) {
    if (!ciphertext || !outPlaintext || ciphertextLen == 0 || (ciphertextLen % 16) != 0 || maxOutLen < ciphertextLen) {
        return 0;
    }

    uint8_t roundKeys[176];
    aesKeyExpansion(key, roundKeys);

    uint8_t prevBlock[16];
    std::memcpy(prevBlock, iv, 16);

    for (size_t offset = 0; offset < ciphertextLen; offset += 16) {
        uint8_t nextPrev[16];
        std::memcpy(nextPrev, ciphertext + offset, 16);
        std::memcpy(outPlaintext + offset, ciphertext + offset, 16);

        aesDecryptBlock(outPlaintext + offset, roundKeys);
        for (size_t i = 0; i < 16; ++i) {
            outPlaintext[offset + i] ^= prevBlock[i];
        }
        std::memcpy(prevBlock, nextPrev, 16);
    }

    const uint8_t padLen = outPlaintext[ciphertextLen - 1];
    if (padLen == 0 || padLen > 16 || padLen > ciphertextLen) {
        return 0;
    }
    for (size_t i = ciphertextLen - padLen; i < ciphertextLen; ++i) {
        if (outPlaintext[i] != padLen) {
            return 0;
        }
    }

    const size_t plainLen = ciphertextLen - padLen;
    if (plainLen < maxOutLen) {
        outPlaintext[plainLen] = '\0';
    }
    return plainLen;
}

size_t CryptoUtils::base64Encode(const uint8_t* input, size_t inputLen, char* outStr, size_t maxOutLen) {
    const size_t needed = ((inputLen + 2) / 3) * 4 + 1;
    if (!outStr || maxOutLen < needed) {
        return 0;
    }

    size_t j = 0;
    for (size_t i = 0; i < inputLen; i += 3) {
        const uint32_t a = input[i];
        const uint32_t b = (i + 1 < inputLen) ? input[i + 1] : 0;
        const uint32_t c = (i + 2 < inputLen) ? input[i + 2] : 0;
        const uint32_t triple = (a << 16) | (b << 8) | c;

        outStr[j++] = B64_CHARS[(triple >> 18) & 0x3F];
        outStr[j++] = B64_CHARS[(triple >> 12) & 0x3F];
        outStr[j++] = (i + 1 < inputLen) ? B64_CHARS[(triple >> 6) & 0x3F] : '=';
        outStr[j++] = (i + 2 < inputLen) ? B64_CHARS[triple & 0x3F] : '=';
    }
    outStr[j] = '\0';
    return j;
}

size_t CryptoUtils::base64Decode(const char* inputStr, uint8_t* outBytes, size_t maxOutLen) {
    if (!inputStr || !outBytes) {
        return 0;
    }
    const size_t len = std::strlen(inputStr);
    if (len == 0 || (len % 4) != 0) {
        return 0;
    }

    size_t outPos = 0;
    for (size_t i = 0; i < len; i += 4) {
        const int v0 = b64Val(inputStr[i]);
        const int v1 = b64Val(inputStr[i + 1]);
        const int v2 = (inputStr[i + 2] == '=') ? 0 : b64Val(inputStr[i + 2]);
        const int v3 = (inputStr[i + 3] == '=') ? 0 : b64Val(inputStr[i + 3]);
        if (v0 < 0 || v1 < 0 || v2 < 0 || v3 < 0) {
            return 0;
        }
        const uint32_t triple = (static_cast<uint32_t>(v0) << 18) |
                                (static_cast<uint32_t>(v1) << 12) |
                                (static_cast<uint32_t>(v2) << 6) |
                                static_cast<uint32_t>(v3);

        if (outPos < maxOutLen) outBytes[outPos++] = static_cast<uint8_t>((triple >> 16) & 0xFF);
        if (inputStr[i + 2] != '=' && outPos < maxOutLen) outBytes[outPos++] = static_cast<uint8_t>((triple >> 8) & 0xFF);
        if (inputStr[i + 3] != '=' && outPos < maxOutLen) outBytes[outPos++] = static_cast<uint8_t>(triple & 0xFF);
    }
    return outPos;
}

bool CryptoUtils::decryptZeroconfPayload(const char* deviceKey,
                                         const char* ivBase64,
                                         const char* dataBase64,
                                         char* outJson,
                                         size_t maxOutJson) {
    if (!deviceKey || !ivBase64 || !dataBase64 || !outJson || maxOutJson < 16) {
        return false;
    }

    uint8_t iv[24] = {0};
    if (base64Decode(ivBase64, iv, sizeof(iv)) != 16) {
        return false;
    }

    uint8_t cipherBuf[512] = {0};
    const size_t cipherLen = base64Decode(dataBase64, cipherBuf, sizeof(cipherBuf));
    if (cipherLen == 0 || (cipherLen % 16) != 0) {
        return false;
    }

    uint8_t key[16];
    deriveLanKey(deviceKey, key);

    uint8_t plainBuf[512] = {0};
    const size_t plainLen = aes128CbcDecryptPkcs7(key, iv, cipherBuf, cipherLen, plainBuf, sizeof(plainBuf));
    if (plainLen == 0 || plainLen >= maxOutJson) {
        return false;
    }

    std::memcpy(outJson, plainBuf, plainLen);
    outJson[plainLen] = '\0';
    return true;
}

bool CryptoUtils::encryptZeroconfPayload(const char* deviceKey,
                                         const char* ivBase64,
                                         const char* plainJson,
                                         char* outDataBase64,
                                         size_t maxOutDataBase64) {
    if (!deviceKey || !ivBase64 || !plainJson || !outDataBase64) {
        return false;
    }

    uint8_t iv[24] = {0};
    if (base64Decode(ivBase64, iv, sizeof(iv)) != 16) {
        return false;
    }

    uint8_t key[16];
    deriveLanKey(deviceKey, key);

    uint8_t cipherBuf[512] = {0};
    const size_t cipherLen = aes128CbcEncryptPkcs7(
        key, iv, reinterpret_cast<const uint8_t*>(plainJson), std::strlen(plainJson), cipherBuf, sizeof(cipherBuf));
    if (cipherLen == 0) {
        return false;
    }

    return base64Encode(cipherBuf, cipherLen, outDataBase64, maxOutDataBase64) > 0;
}

} // namespace gps::app


#ifndef APPLICATION_CRYPTOUTILS_H
#define APPLICATION_CRYPTOUTILS_H

#include <cstddef>
#include <cstdint>

namespace gps::app {

/**
 * @brief Primitivas criptográficas puras C++17 para tramas locales cifradas:
 * - Derivación de clave de 16 bytes mediante MD5(deviceKey)
 * - Cifrado y descifrado AES-128-CBC con relleno PKCS#7
 * - Codificación y decodificación Base64 (RFC 4648)
 */
class CryptoUtils {
public:
  static void computeMd5(const uint8_t *data, size_t len,
                         uint8_t outDigest[16]);
  static void deriveLanKey(const char *deviceKey, uint8_t outKey[16]);

  static size_t aes128CbcEncryptPkcs7(const uint8_t key[16],
                                      const uint8_t iv[16],
                                      const uint8_t *plaintext,
                                      size_t plaintextLen,
                                      uint8_t *outCiphertext, size_t maxOutLen);

  static size_t aes128CbcDecryptPkcs7(const uint8_t key[16],
                                      const uint8_t iv[16],
                                      const uint8_t *ciphertext,
                                      size_t ciphertextLen,
                                      uint8_t *outPlaintext, size_t maxOutLen);

  static size_t base64Encode(const uint8_t *input, size_t inputLen,
                             char *outStr, size_t maxOutLen);
  static size_t base64Decode(const char *inputStr, uint8_t *outBytes,
                             size_t maxOutLen);

  static bool decryptZeroconfPayload(const char *deviceKey,
                                     const char *ivBase64,
                                     const char *dataBase64, char *outJson,
                                     size_t maxOutJson);

  static bool encryptZeroconfPayload(const char *deviceKey,
                                     const char *ivBase64,
                                     const char *plainJson,
                                     char *outDataBase64,
                                     size_t maxOutDataBase64);
};

} // namespace gps::app

#endif // APPLICATION_CRYPTOUTILS_H


#ifndef BASE64_H
#define BASE64_H

#include <string>
#include <vector>

namespace sensors {
namespace gnss {

/**
 * @brief Encode binary data as base64 string
 *
 * @param data Pointer to data to encode
 * @param length Length of data in bytes
 * @return Base64 encoded string
 */
std::string base64_encode(const unsigned char* data, size_t length);

/**
 * @brief Decode base64 string to binary data
 *
 * @param encoded Base64 encoded string
 * @return Decoded binary data
 */
std::vector<unsigned char> base64_decode(const std::string& encoded);

}  // namespace gnss
}  // namespace sensors

#endif  // BASE64_H
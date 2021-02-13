#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"
#include <iostream>
#include <fstream>

class Sha256IdGenerator : public IdGenerator {
  public:
    virtual PageId generateId(std::string const &content) const {
      char buffer[64];
      std::string hashValue;
      std::fstream contentFile("content.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);

      if (!contentFile) {
        throw std::runtime_error("Could not open a file.");
      }

      contentFile << content;

      FILE *pipe = popen("sha256sum content.txt", "r");

      if (pipe == nullptr) {
        throw std::runtime_error("popen() failed!");
      }

      if (fgets(buffer, 64, pipe) != nullptr) {
        for (int i = 0; i < 64; i++) {
          printf("%c ", buffer[i]);
        }

        hashValue = buffer;
      } else {
        throw std::runtime_error("fgets() failed!");
      }

      pclose(pipe);
      contentFile.close();

      if (remove("content.txt") != 0) {
        throw std::runtime_error("Removal of a file failed!");
      }

      return PageId(hashValue);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */

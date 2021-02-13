#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class Sha256IdGenerator : public IdGenerator {
  public:
    virtual PageId generateId(std::string const &content) const {
      char buffer[64];
      std::string hashValue;
      std::fstream contentFile("content.txt");

      contentFile << content;

      FILE *pipe = popen("sha256sum content.txt", "r");

      if (pipe == nullptr) {
        throw std::runtime_error("popen() failed!");
      }

      if (fgets(buffer, 64, pipe) != nullptr) {
        hashValue = buffer;
      } else {
        throw std::runtime_error("fgets() failed!");
      }

      pclose(pipe);
      contentFile.close();

      /*    if (remove("content.txt") != 0) {
            throw std::runtime_error("Removal of a file failed!");
          }*/

      return PageId(hashValue);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */

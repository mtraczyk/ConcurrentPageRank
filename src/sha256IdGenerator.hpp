#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"
#include <iostream>
#include <fstream>

class Sha256IdGenerator : public IdGenerator {
  public:
    virtual PageId generateId(std::string const &content) const {
      char buffer[65];
      std::string hashValue;
      std::ofstream contentFile("content.txt");

      contentFile << content;

      std::system("cat content.txt | tr -d '\n' > output.txt");

      FILE *pipe = popen("sha256sum output.txt", "r");

      if (pipe == nullptr) {
        throw std::runtime_error("popen() failed!");
      }

      if (fgets(buffer, 64, pipe) != nullptr) {
        buffer[64] = '\0';
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

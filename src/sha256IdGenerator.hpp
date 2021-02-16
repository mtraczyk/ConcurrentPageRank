#ifndef SRC_SHA256IDGENERATOR_HPP_
#define SRC_SHA256IDGENERATOR_HPP_

#include "immutable/idGenerator.hpp"
#include "immutable/pageId.hpp"
#include <iostream>
#include <fstream>
#include <atomic>

class Sha256IdGenerator : public IdGenerator {
  public:
    virtual PageId generateId(std::string const &content) const {
      char buffer[64];
      std::string hashValue;
      static std::atomic<int> counter(0);
      std::string fileName = "content" + std::to_string(counter.fetch_add(1)) + ".txt";
      std::fstream contentFile(fileName, std::ios::in | std::ios::out | std::ios::trunc);

      if (!contentFile) {
        throw std::runtime_error("Could not open a file.");
      }

      contentFile << content;
      contentFile.close();

      std::string aux = "sha256sum " + fileName;
      FILE *pipe = popen(aux.c_str(), "r");

      if (pipe == nullptr) {
        throw std::runtime_error("popen() failed!");
      }

      if (fscanf(pipe, "%64s", buffer) == 1) {
        hashValue = buffer;
      } else {
        throw std::runtime_error("fgets() failed!");
      }

      pclose(pipe);

      if (remove(fileName.c_str()) != 0) {
        throw std::runtime_error("Removal of a file failed!");
      }

      return PageId(hashValue);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
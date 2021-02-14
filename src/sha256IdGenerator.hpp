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
      std::string aux = "echo tr -d '\n' | echo '" + content + "' | sha256sum";
      FILE *pipe = popen(aux.c_str(), "r");

      if (pipe == nullptr) {
        throw std::runtime_error("popen() failed!");
      }

      if (fscanf(pipe, "%64s", buffer) == 1) {
        hashValue = buffer;
      } else {
        throw std::runtime_error("fscanf() failed!");
      }

      pclose(pipe);

      return PageId(hashValue);
    }
};

#endif /* SRC_SHA256IDGENERATOR_HPP_ */
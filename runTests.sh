#!/bin/bash
set -e

echo ">>>>>>>>> Run clang-format"
FILES=$(find | grep "hpp$\|cpp$" | grep -v CMake)
clang-format -i -style=WebKit $FILES

#clang-tidy $FILES # not available on students?

echo ">>>>>>>>> Run tests in DEBUG"
# Run tests in debug config
cmake -DCMAKE_BUILD_TYPE=Debug .
make

/usr/bin/valgrind valgrind --error-exitcode=123 --leak-check=full ./tests/pageRankCalculationTest

./tests/sha256Test

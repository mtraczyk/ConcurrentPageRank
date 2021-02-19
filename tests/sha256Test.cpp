
#include "../src/immutable/common.hpp"

#include "../src/sha256IdGenerator.hpp"

void testSha256(std::string const &testScenario, std::string const &expectedResult) {
  Sha256IdGenerator generator;
  PageId result = generator.generateId(testScenario);
  ASSERT(result == PageId(expectedResult),
         "Incorrect SHA256, scenario=" << testScenario
                                       << ", result=" << result
                                       << ", expectedResult=" << expectedResult);
}

int main() {
  testSha256("Ala ma kota\n", "c51bc001db0206126e1681ba88497ce583f077a92e427e4f62da96b691d28813");
  testSha256("Zawartość naprawdę naprawdę naprawdę wielkiego pliku 1234567890\n",
             "69bddbdc52992ae9952d3368d48bfe0517ce346d6040e495de3542926294498b");
  testSha256("abc\"\n", "d9fb3785c8eab50a78de18ac4acd3be43ceade0a2193d6346f9b656d2d818700");
  testSha256("abc\'\n", "7b963e8f05bccdc7c2b556356d70769a2c28a6f32794d77dfa44d8c4ee1a048d");
  testSha256("\";vim;\"", "7bf3d6ee225efb5369fcd0bad0bbb7bd0ddd150d7a7ff0a66463be1c12d41ee9");
  testSha256("Ala ma kota", "124bfb6284d82f3b1105f88e3e7a0ee02d0e525193413c05b75041917022cd6e");
  testSha256("Ala\nma\nkota", "c824ad9ef06c38fcf6ed303013bb8d105407aa66b1d4848d811637027d6cd001");

  return 0;
}

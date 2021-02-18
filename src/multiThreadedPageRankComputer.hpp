#ifndef SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_
#define SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_

#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <future>

#include "immutable/network.hpp"
#include "immutable/pageIdAndRank.hpp"
#include "immutable/pageRankComputer.hpp"

namespace {
  void initializePagesIds(uint32_t numThreads, uint32_t myNumber, Network const &network) {
    /* Generating names, the work is distributed between threads.
     * The distribution might not be very even. Assuming that network`s
     * pages have similar size of content such a solution (timewise) should work very well.
     * Also if there are many pages and their content is rather small, it should work much better
     * than distributing pages using some shared data which would require synchronization.
    */
    auto const &pages = network.getPages();
    auto const &generator = network.getGenerator();
    for (uint32_t i = myNumber; i < network.getSize(); i += numThreads) {
      pages[i].generateId(generator);
    }
  }

  void initializeStructures(Network const &network, uint32_t numThreads,
                            std::unordered_map<PageId, PageRank, PageIdHash> &pageHashMap,
                            std::unordered_map<PageId, uint32_t, PageIdHash> &numLinks,
                            std::vector<PageId> &danglingNodes,
                            std::unordered_map<PageId, std::vector<PageId>, PageIdHash> &edges) {
    std::vector<std::thread> threadsVector;

    for (uint32_t i = 0; i < numThreads; i++) {
      // Initializing pages` ids.
      threadsVector.push_back(std::thread{initializePagesIds, numThreads, i, std::ref(network)});
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      threadsVector[i].join();
    }

    // Initialization of structures needed in MultiThreadedPageRankComputer::computeForNetwork.
    for (auto const &page : network.getPages()) {
      pageHashMap[page.getId()] = 1.0 / network.getSize();
    }

    for (auto const &page : network.getPages()) {
      numLinks[page.getId()] = page.getLinks().size();
    }

    for (auto const &page : network.getPages()) {
      if (page.getLinks().size() == 0) {
        danglingNodes.push_back(page.getId());
      }
    }

    for (auto const &page : network.getPages()) {
      for (auto const &link : page.getLinks()) {
        edges[link].push_back(page.getId());
      }
    }
  }

  /* One iteration of an algorithm is evenly distributed between numThreads.
   * Every thread during one iteration is gonna invoke this function.
  */
  void pageRankIteration(Network const &network, double alpha, double dangleSum,
                         uint32_t threadNum, uint32_t numThreads,
                         std::unordered_map<PageId, PageRank, PageIdHash> &pageHashMap,
                         std::unordered_map<PageId, uint32_t, PageIdHash> &numLinks,
                         std::unordered_map<PageId, PageRank, PageIdHash> &previousPageHashMap,
                         std::unordered_map<PageId, std::vector<PageId>, PageIdHash> &edges,
                         std::promise<double> &differencePromise) {
    double difference = 0;
    auto ptrToPages = &network.getPages();
    for (uint32_t i = threadNum; i < network.getSize(); i += numThreads) {
      PageId pageId = (*ptrToPages)[i].getId();

      double danglingWeight = 1.0 / network.getSize();
      pageHashMap[pageId] = dangleSum * danglingWeight + (1.0 - alpha) / network.getSize();

      if (edges.count(pageId) > 0) {
        for (auto const &link : edges[pageId]) {
          pageHashMap[pageId] += alpha * previousPageHashMap[link] / numLinks[link];
        }
      }
      difference += std::abs(previousPageHashMap[pageId] - pageHashMap[pageId]);
    }

    differencePromise.set_value(difference);
  }

  // Auxiliary function that sums all of the differenceFutures obtained by threads.
  double summaryDifference(uint32_t numThreads, std::future<double> *differenceFutures) {
    double difference = 0;

    for (uint32_t i = 0; i < numThreads; i++) {
      difference += differenceFutures[i].get();
    }

    return difference;
  }

  void singleThreadDangleSum(std::vector<PageId> const &danglingNodes, uint32_t threadNum, uint32_t numThreads,
                             std::promise<double> &dangleSumPromise,
                             std::unordered_map<PageId, PageRank, PageIdHash> &previousPageHashMap) {
    double dangleSum = 0;

    for (uint32_t i = threadNum; i < danglingNodes.size(); i += numThreads) {
      dangleSum += previousPageHashMap[danglingNodes[i]];
    }

    dangleSumPromise.set_value(dangleSum);
  }

  // Function which obtains dangleSum using numThreads.
  double countDangleSum(std::vector<PageId> const &danglingNodes, uint32_t numThreads,
                        std::unordered_map<PageId, PageRank, PageIdHash> &previousPageHashMap) {
    double dangleSum = 0;
    std::promise<double> dangleSumPromises[numThreads];
    std::future<double> dangleSumFutures[numThreads];
    std::vector<std::thread> threadsVector;

    for (uint32_t i = 0; i < numThreads; i++) {
      dangleSumFutures[i] = dangleSumPromises[i].get_future();
      threadsVector.push_back(std::thread{singleThreadDangleSum, std::ref(danglingNodes), i, numThreads,
                                          std::ref(dangleSumPromises[i]), std::ref(previousPageHashMap)});
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      dangleSum += dangleSumFutures[i].get();
    }

    for (auto &thread : threadsVector) {
      thread.join();
    }

    return dangleSum;
  }
};

class MultiThreadedPageRankComputer : public PageRankComputer {
  public:
    MultiThreadedPageRankComputer(uint32_t numThreadsArg)
      : numThreads(numThreadsArg) {};

    std::vector<PageIdAndRank>
    computeForNetwork(Network const &network, double alpha, uint32_t iterations,
                      double tolerance) const {
      /* The implementation is basically the same as singleThreadedPageRankComputer::computeForNetwork.
       * The only difference is that the work, is distributed between threads.
      */
      std::vector<std::thread> threadsVector;
      std::unordered_map<PageId, PageRank, PageIdHash> pageHashMap;
      std::unordered_map<PageId, uint32_t, PageIdHash> numLinks;
      std::vector<PageId> danglingNodes;
      std::unordered_map<PageId, std::vector<PageId>, PageIdHash> edges;

      initializeStructures(network, numThreads, pageHashMap, numLinks, danglingNodes, edges);

      for (uint32_t i = 0; i < iterations; i++) {
        std::unordered_map<PageId, PageRank, PageIdHash> previousPageHashMap = pageHashMap;
        threadsVector.clear();
        std::promise<double> differencePromises[numThreads];
        std::future<double> differenceFutures[numThreads];

        double dangleSum = countDangleSum(danglingNodes, numThreads, previousPageHashMap) * alpha;

        for (uint32_t j = 0; j < numThreads; j++) {
          differenceFutures[j] = differencePromises[j].get_future();
          threadsVector.push_back(std::thread{pageRankIteration, std::ref(network), alpha, dangleSum, j, numThreads,
                                              std::ref(pageHashMap), std::ref(numLinks),
                                              std::ref(previousPageHashMap), std::ref(edges),
                                              std::ref(differencePromises[j])});
        }

        if (summaryDifference(numThreads, differenceFutures) < tolerance) {
          std::vector<PageIdAndRank> result;

          for (auto const &iter : pageHashMap) {
            result.push_back(PageIdAndRank(iter.first, iter.second));
          }

          for (auto &thread : threadsVector) {
            thread.join();
          }

          ASSERT(result.size() == network.getSize(),
                 "Invalid result size=" << result.size() << ", for network" << network);

          return result;
        }

        for (auto &thread : threadsVector) {
          thread.join();
        }
      }

      ASSERT(false, "Not able to find result in iterations=" << iterations);
    }

    std::string getName() const {
      return "MultiThreadedPageRankComputer[" + std::to_string(this->numThreads) + "]";
    }

  private:
    uint32_t numThreads;
};

#endif /* SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_ */
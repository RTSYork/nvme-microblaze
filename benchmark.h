#ifndef SRC_BENCHMARK_H_
#define SRC_BENCHMARK_H_

int read_benchmark(int pci, int nsid, u64 mem_base_pci, void *mem_base_mb, size_t mem_size);
int write_benchmark(int pci, int nsid, u64 mem_base_pci, void *mem_base_mb, size_t mem_size);

#endif /* SRC_BENCHMARK_H_ */

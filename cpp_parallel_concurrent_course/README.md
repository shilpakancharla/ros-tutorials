# Parallel and Concurrent Programming with C++

## Parallel Computing Hardware

### Sequential versus Parallel Computing

* Serial execution: program broken down into series of instructions; can only execute one instruction at a time. Limitations: speed of processor determines how quickly program to execute.
* Parallel execution: break down instructions into independent parts that can be processed by different processors, coordinate steps. Doesn't necessarily means it will run a lot faster, but must have element of coordination and communication. 
* When done right, parallel execution increases throughput: accomplish a single task fast; accomplish more tasks in a given time.

### Parallel Computing Hardware

* Flynn's Taxonomy - 4 types, dependent on number of instruction streams and number of data streams.
1. Single Instruction Single Data (SISD): sequential computer with a single processor unit (only act on one data at a time)
2. Single Instruction Multiple Data (SIMD): parallel computer with multiple processing units; execute same instructions on different data
3. Multiple Instruction Single Data (MISD): each processing unit executes its own instructions, but operate on same set of data (not as practical)
4. Multiple Instruction Multiple Data (MIMD): every processing unit executes different set of instructions on different sets of data (most commonly used)

* MIMD can be further divided into two categories:
1. Single Program, Multiple Data (SPMD): multiple processing execute same program (not necessarily same instruction within program) on different data simultaneously; usually includes conditional logic to only execute certain parts
2. Multiple Program, Multiple Data (MPMD): processors executing different, independent programs at the same time and operating on different data; only processor selected as host and others as nodes

### Shared versus Distributed Memory

* Computer memory operates faster than processor memory
* Shared memory
- all processors access the same meory with global address space
* Distributed memory:

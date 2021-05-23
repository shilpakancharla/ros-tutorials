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
* Shared memory: all processors access the same memory with global address space; types are uniform memory access (UMA) and non-uniform memory access (NUMA).
* Uniform memory access (UMA): all of the processors have equal access to memory; most common is Symmetric Multiprocessing (SMP), processors (which have cache memory) connected to main memory via system bus
* Non-uniform memory access (NUMA): some processors have quicker access to some memory than others; but overall, all processors can see everything in memory
* Distributed memory: each processor has its own address in memory space; each processor operates independently and if it makes change to local memory, that is not automatically reflected in other processor memory

## Threads and Processes

* Process: includes code, data, and state information; indpendent instance of a running program; separate memory address
* Thread: independent path of execution; subset of a process; operating system schedules threads for execution
* Threads that belong to same process belong to the same address space, giving access to code and data
* Inter-process Communication (IPC): sockets and pipes, shared memory, and remote procedure calls
* Threads are "lightweight" - require less overhead to create and terminate
* Operating system can switch between threads faster than processes

### Concurrent versus Parallel Execution
* Concurrency: ability of a program to be broken into parts that can run independently of each other; dealing with multiple things at once; program structure
* Parallel hardware: multi-core processors, graphics processing units (GPUs), computer cluster
* Parallelism: simultaneous execution; doing multiple things at once
* I/O Devices: need to execute concurrently; managed by OS as independent processors (useful in concurrent programming tasks)
* Parallel processing useful for computational tasks, like matrix multiplication

### Execution Scheduling
* Scheduler: operating system function that assigns processes and threads to run on available CPUs
* Ready queue: when a process is ready to run, it gets placed in this queue
* Context switch: OS needs to save the state, or context, to be resumed later; loading the saved state for the new process or thread to run
* Scheduling algorithms: some are pre-emptive (may pause), non-preemptive (may run the entire time)
1. First come, first serve
2. Shortest job next
3. Priority
4. Shortest remaining time
5. Round-robin
6. Multiple-level queues

* Scheduling goals: maximize throughput, maximize fairness, minimize wait time, minimize latency

### Thread Life Cycle

* Threads can spawn child threads, and so on. As they complete, they notify the parent thread and terminate. The parent thread is the last one to terminate.
* Once a thread has started, it's in the runnable state.
* When a thread needs to wait in order to run, it's in the blocked state. It will not use any CPU resources. It frees up processor for other threads to use.
* `join()`: wait until another thread complets its execution
* When a thread completes its execution or is abnormally aborted, it enters the terminated state.
* States: new, runnable, blocked, and terminated 

### Detached Thread

* Garbage collector: automatic memory management; reclaims memory no longer in use by program
* Threads that are performing background services (like garbage collection) should run as a detached thread, or else we have to wait forever
* Daemon (background) thread: does not prevent the process from terminating; by default, therads are created as non-daemon

## Mutual Exclusion

### Data Race

* Data race: problem that occurs when two or more concurrent threads access the same memory location and at least one thread is modifying it
* Using synchronization techniques to prevent against this

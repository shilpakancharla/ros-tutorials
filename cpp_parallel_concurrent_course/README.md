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
* Pay attentionwhenever two or more threads access the same resource to prevent data races

### Mutual Exclusion

* Critical section: code segment that accesses a shared resource; should not be executed by more than one thread or process at a time
* Mutex (lock): mechanism to implement mutual exclusion; only one thread or process can posesses at a time; limits access to critical section
* Atomic operations: execute as a single action, relative to other threads; cannot be interrupted by other concurrent threads
* Acquiring a lock: if a lock is already taken, block/wait for it to be available
* Keep protected sections of code as short as possible

## Locks

### Recursive Mutex

* If I attempt to lock the mutex while another thread has it, my thread will be blocked and I need to wait until it unlocks it so it will become available.
* If I attempt to lock the mutex, it doesn't appear to be available so my thread will just have to wait too. 
* If a thread tries to lock a mutex that it's already locked, it'll enter into a waiting list for that mutex, which results in something called a deadlock because no other thread can unlock that mutex. There may be times when a program needs to lock a mutex multiple times before unlocking it.
* Reetrant mutex: mutex that can be locked multiple times by the same process or thread. Keeps track of how many times it's been locked by othe owning thread and it has to be unlocked an equal number of time before another thread can lock it.
* If you don't unlock the reentrant mutex the same number of times. you can still end up stuck.
* If you later create another function that uses the same mutext to protect some other section of code, and that section of code uses the increment counter function, since those functions are nested, it ends up locking the mutex twice before unlocking it.
* Common terms: reentrant mutex, reentrant lock, recursive mutex, recursive lock
 
### Try Lock

* Non-blocking lock/acquire method for mutex
* If the mutex is available, lock it and return TRUE
* If the mutex is not available, immediately return FALSE

### Shared Mutex

* Reader-Writer Lock
* Shared read: multiple threads at once, or
* Exclusive write: only one thread at a time
* Keep track of number of threads being read versus number of threads being writing

## Liveness

### Deadlock

* Deadlock: each member is waiting for another member to take action
* Liveness: properties that require a system to make progress; members may have to "take turns" in critical sections

### Abandoned Lock

* Errors occurring when a thread traps some resource (a critical section, mutex) and is withdrawn from execution for some reason.

### Starvation

* A process or thread is perpeutally denied the resources it needs.

### Livelock

* Multiple threads or processes are actively responding to each other to resolve conflict, but that prevents them from making progress

## Coarse-Grained Parallelism

* Small number of large tasks
* Advantage - high computation-to-communication ratio
* Disadvantage - inefficient load balancing

## Synchronization

### Condition Variable

* Condition variable: queue of threads waiting for a certain condition; associated with a mutex
* Monitor: protect section of code with mutual exclusion; provide ability for threads to wait until a condition occurs
* Three Operations:
1. `wait`: automatically release lock on the mtuex; go to sleep and enter waiting queue; reacquire lock when woken up
2. `signal`: wake up one thread from condition variable queue; also called notify or wake
3. `broadcast`: wake up all threads from condition variable queue; also called notify all or wake all
* Shared Queue or Buffer: mutex, condition variables (`BufferNotFull`, `BufferNotEmpty`)
* Using a condition variable:

````
std::unique_lock<std::mutex> lk(cv_m);

while !(SOME_CONDIITON) {
 cv.wait(lk); // Wait here until signaled
}

// Execute critical section
````

### Producer-Consumer

* Producer(s): add elements to shared data structure
* Consumer(s): remove elements from shared data structure
* First-in-first-out (FIFO): items are removed in the same order that they're added to the queue; first item added will be first item removed
* Synchronization challenges: enforce mutual exclusion of producers and consumers; prevent producers from trying to add data to a full queue; prevent consumers from trying to remove data from an empty queue
* Unbounded queue: queue with unlimited capacity (still limited by memory)
* Average rate of production < Average rate of consumption
* Pipeline: chain of processing elements

### Semaphore

* Synchronization mechanism
* Can be used by multiple threads at the same time
* Includes a counter to track avaialability
* `acquire()`: if counter is positive, decrement counter; if counter is zero, wait until avaialble
* `release()`: increment counter, signal waiting thread
* Counting sempahore: value greater than or equal to 0; used to track limited resources (pool of connections, items in a queue)
* 0 is a locked state
* 1 represents unlocked state
* Semaphore used similar to mutex by acquiring/releasing
* Mutex vs Semaphore: mutex can only be acquired/released by the same thread; semaphore acquired/release by different threads

## Barriers

### Race Conditions

* Searching for Race Conditions: use sleep statements to modify and execution order
* Heisenbug: a software bug that disappears when you try to study it
* Data races and race conditions are two different potential problesm in concurrent programs 
* Data races: can occur when two or more threads concurrently access the same memory location
* Race condition: flaw in the timing or ordering of a program's execution that causes incorrect behavior

### Barrier

* Barrier: prevents a group of threads from proceeding until enough threads have reached the barrier

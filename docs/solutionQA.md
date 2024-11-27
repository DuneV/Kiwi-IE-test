# Answers

### C++: Why do we use `UniquePointer` instead of `SharedPointer` to publish a ROS2 message?
Using `UniquePointer` ensures that the published message is managed by a single owner, which improves performance in terms of memory management and reduces the overhead due to reference counting that comes from `SharedPointer`. ROS2 takes ownership of `UniquePointer` which avoids unnecessary copies and ensures safety in terms of unique management (it follows a bit of the immutability principle once it has an owner). On the contrary, `SharedPointer` would require additional bookkeeping for shared ownership which is unnecessary when we talk about topics, since ROS2 allows access to this **data*** via shared nodes or networks.

---

### Logic: What advantages and disadvantages does the queue structure have over a vector (C++) or a list (Python)?

### Advantages:
- **FIFO order:** Queues guarantee first-in-first-out processing, making them ideal for task scheduling and sequential processing as done here (batch analysis in the case of my node). - **Specialized interface:** Queues have simplified specialized functions (`push`, `pop`, `front`, `back`) designed for sequential data handling making interaction with them easier.

- **Thread-safe implementations:** Many queue implementations are thread-safe or mask-safe by default, unlike vectors or lists which are easily exploitable (more so in c++ for indexing overrun exceptions).

### Disadvantages:
- **Restricted access:** Queues do not allow random access or iteration through elements like vectors or lists, as they are dynamic.
- **Less flexible:** Queues are limited to front-end and back-end operations, while arrays and lists provide more comprehensive operations, with functions implemented in the python case (`sort`, `get`).

---

### Logic: Why do we use a variable `m_multi_sound`? Please explain...
Reviewing the `speaker.cpp` file it acts as a flag for ambient sound playback. Essentially:
- **Prevents conflicts** between ambient sound playback and the requested track, ensuring they are not played simultaneously, in the case that is (m_multi_sound=0) disables their playback.
- **Controls transitions** between different types of sounds (ambient and requested track) in an orderly manner.
- **Allows to stop and resume** sound playback in a controlled manner, to stop and enable it.

---

### Docker: Explain in your own words what the `apt-get autoremove && apt-get clean -y` command is for?
- **`apt-get autoremove`:** Removes unnecessary packages that were automatically installed as dependencies but are no longer needed (although this only works in system setup since sometimes the system identifies ROS itself as an unnecessary dependency so it is not recommended to abuse the command). This helps free up disk space and keep the system clean.
- **`apt-get clean -y`:** Removes cached package files in the `/var/cache/apt/archives` folder to reduce the image size in Docker layers, making it more compact.

---

### Docker: If you modify a layer, what happens to the previous and next ones?
You can
- **Previous layers:** They remain unchanged because Docker caches them. They are generally reused.
- **Next layers:** These are invalidated and rebuilt, as their content depends on the modified layer. This ensures consistency and avoids obsolete configurations, although in parallel builds it can cause issues regarding the type of modification if the layer is directly incomplete.

---

### Docker: Can we change the base image (`FROM ubuntu:22.04`) from the Dockerfile to another one?
Yes, it is possible to change the base image in the Dockerfile.
However:
1. All subsequent instructions may need adjustments to accommodate differences in the new base image - a clear case here being ROS2 versions that are not compatible with certain versions or the case of images that have CUDA-type dependencies for each graphics/system/driver.
2. Dependencies and functionality need to be re-evaluated for compatibility and more importantly stability.

---

### Python: Why should we use a thread to spin up the node?
Using a thread to spin up the node allows the main program to continue executing other tasks simultaneously. This is particularly useful in multi-threaded applications where the node needs to process incoming messages and callbacks without blocking the main execution flow but I have to accept that I prefer the way that ROS(1) did that because it easier to management the frequency of the nodes. Ensures better performance and responsiveness in real-time systems

```sh
ros2 action send_goal --feedback /navigate_to_goal amr_interfaces/action/NavigateToGoal "{
  goal_gps: [50.7154152, 10.4678595, 0.0],
  lanelet_ids: [-6778, -6802, -6779, -6782, -6768, -6788, -6793, -6777, -6784, -6776],
  is_inverted: [False, False, False, False, False, False, False, True, False, False],
  total_distance: 147.2242238312887,
  estimated_time: 9.054729203596866
}"
```

**self.planning_lock = Lock()** explained:
```plaintext
planning_loop called
│
├─ Is lock free? Yes (first time)
│  │
│  ├─ acquire() returns True
│  │  │
│  │  ├─ Continue method
│  │  │  │
│  │  │  ├─ Do planning work...
│  │  │  │
│  │  │  ├─ Finally: release() lock
│  │  │
│  │
│
planning_loop called again
│
├─ Is lock free? No (already acquired)
│  │
│  ├─ acquire() returns False because we have passed the blocking=False param. If blocking=True then we would wait till the previous planning_loop's lock is released and we don't want to do that.
│  │  │
│  │  ├─ Return immediately
│  │  │  (planning is skipped)
```

Why using Lock() instead of a flag like is_planning=True

# Lock vs. Boolean Flag: Understanding the Difference

You're right that both approaches can prevent concurrent execution, but there are important differences between using a `Lock()` vs. a simple boolean flag (`is_planning`).

## Why Use a Lock Instead of a Boolean Flag

The key downside of using just a boolean flag is **race conditions**. In concurrent systems, race conditions happen when the behavior depends on the sequence or timing of uncontrollable events.

### Race Condition Example

Let's see how a race condition could occur with a boolean flag:

```python
def planning_loop(self):
    # Check if planning is already happening
    if self.is_planning:
        return
        
    # Set flag to true - WE THINK we're protected now
    self.is_planning = True  
    
    # ... planning code ...
    
    # Reset flag when done
    self.is_planning = False
```

Problem: Between checking `if self.is_planning` and setting `self.is_planning = True`, another thread could execute exactly the same code. Both would see `is_planning == False` and both would proceed.

### Thread Preemption Issues

The issue gets worse with preemptive multithreading:

1. Thread A checks `if not self.is_planning` (it's False)
2. Thread A gets preempted (paused) by the OS before setting the flag
3. Thread B executes and also checks `if not self.is_planning` (still False!)
4. Thread B sets `self.is_planning = True` and proceeds
5. Thread A resumes, sets `self.is_planning = True` again, and also proceeds
6. Now both are executing the critical section

## Benefits of Using a Lock

A `Lock` object provides atomic operations:

1. **Atomicity**: `acquire()` is executed as a single, uninterruptible operation
2. **Guaranteed Mutual Exclusion**: Only one thread can hold the lock at a time
3. **OS-level Synchronization**: Locks use low-level OS mechanisms for thread safety

## Why Non-Blocking Acquire is Used

Your code uses `acquire(blocking=False)`, which:

1. Returns `True` if lock acquired, `False` if already locked
2. Never blocks/waits, returning immediately
3. Avoids potential deadlocks in callback-based systems




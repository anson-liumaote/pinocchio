import threading
import time

condition = threading.Condition()

def waiter():
    while 1:
        with condition:  # Acquire the Condition lock
            print("Waiting for the condition...")
            condition.wait(timeout=10)  # Wait with a timeout
            print("Condition met or timeout occurred, proceeding...")
        
        # Continue processing without blocking the Condition
            print("Processing after wait...")
            time.sleep(1)
    # Your processing logic here

def notifier():
    while 1:
        time.sleep(5)  # Simulate some delay
        with condition:  # Acquire the Condition lock
            print("Notifying waiting thread...")
            condition.notify()  # Notify the waiting thread

# Example usage
wait_thread = threading.Thread(target=waiter)
notify_thread = threading.Thread(target=notifier)

wait_thread.start()
notify_thread.start()

wait_thread.join()
notify_thread.join()

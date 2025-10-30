"""
orientation_mediator.py

Implements a thread-safe publish/subscribe mediator class.
This allows the EKF estimator to publish its data to one or
more subscribers (e.g., a visualizer, a logger) without
being directly coupled to them.
"""

import threading

class OrientationMediator:
    """
    A thread-safe Pub/Sub class for distributing orientation data.
    """
    def __init__(self):
        self._subscribers = []
        self._lock = threading.Lock()

    def subscribe(self, callback):
        """
        Adds a new subscriber (a callback function) to the list.
        
        Args:
            callback (function): A function that takes one argument,
                                 the OrientationData object.
        """
        with self._lock:
            if callback not in self._subscribers:
                self._subscribers.append(callback)
        print(f"Mediator: New subscriber added: {callback.__name__}")

    def publish(self, data):
        """
        Publishes data to all registered subscribers.
        
        Args:
            data (OrientationData): The data object to send.
        """
        with self._lock:
            for callback in self._subscribers:
                try:
                    callback(data)
                except Exception as e:
                    # Don't let one bad subscriber stop the others
                    print(f"Error in subscriber {callback.__name__}: {e}")

"""
orientation_mediator.py

Implements a thread-safe publish/subscribe mediator class.
This allows different components (handlers, estimators, visualizers)
to communicate without being directly coupled.
"""

import threading

class Mediator:
    """
    A thread-safe Pub/Sub class for distributing data via topics.
    """
    def __init__(self):
        self._topics = {}
        self._lock = threading.Lock()
        print("Mediator: System initialized.")

    def subscribe(self, topic, callback):
        """
        Adds a new subscriber (a callback function) to a topic.
        
        Args:
            topic (str): The name of the topic (e.g., "sensor", "orientation").
            callback (function): A function that takes one argument (the data).
        """
        with self._lock:
            if topic not in self._topics:
                self._topics[topic] = []
                
            if callback not in self._topics[topic]:
                self._topics[topic].append(callback)
                
        print(f"Mediator: New subscriber to '{topic}': {callback.__name__}")

    def publish(self, topic, data):
        """
        Publishes data to all subscribers of a specific topic.
        
        Args:
            topic (str): The topic to publish to.
            data: The data object to send.
        """
        subscribers_to_notify = []
        with self._lock:
            if topic in self._topics:
                # Make a copy to avoid holding the lock during callbacks
                subscribers_to_notify = self._topics[topic][:]

        if not subscribers_to_notify:
            # print(f"Mediator: No subscribers for topic '{topic}'")
            return

        for callback in subscribers_to_notify:
            try:
                callback(data)
            except Exception as e:
                # Don't let one bad subscriber stop the others
                print(f"Error in subscriber {callback.__name__} for topic '{topic}': {e}")
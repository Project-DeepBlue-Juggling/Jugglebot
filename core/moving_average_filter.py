class MovingAverageFilter:
    def __init__(self, window_size=10):
        # Window size for the moving average
        self.window_size = window_size
        
        # Buffers for each of the six inputs
        self.buffers = [[] for _ in range(6)]
        
    def update_and_filter(self, inputs):
        """Update the buffers with new data and return the moving average."""
        averages = []
        
        for i, value in enumerate(inputs):
            # Append the new value to the buffer
            self.buffers[i].append(value)
            
            # If the buffer exceeds the window size, remove the oldest value
            if len(self.buffers[i]) > self.window_size:
                self.buffers[i].pop(0)
            
            # Compute and store the average of the buffer
            averages.append(sum(self.buffers[i]) / len(self.buffers[i]))
        
        return averages

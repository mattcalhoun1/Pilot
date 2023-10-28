class ActionBase:
    def get_name (self):
        return "Base"

    def execute (self, params):
        pass

    # whether to record the task as complete, prior to completing it.
    # this is important for multitasking or shutting down the device
    def mark_complete_immediately (self):
        return False
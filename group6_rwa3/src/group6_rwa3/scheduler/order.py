class OrderReceived:
    """Order which we received through a callback of Order topic
    """

    # pylint: disable=too-few-public-methods

    def __init__(self, order):
        self.order = order

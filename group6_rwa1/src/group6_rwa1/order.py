from nist_gear.msg import Order

class OrderReceived:
    def __init__(self, order):
        self.order = order

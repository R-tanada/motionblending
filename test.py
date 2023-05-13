class test:
    test = 100
    def __init__(self, num) -> None:
        self.num = num

    def hello(self):
        self.test = self.num

test1 = test(30)
test2 = test(60)

print(test1.test, test2.test, test.test)

test1.hello()
test2.hello()

print(test1.test, test2.test, test.test)
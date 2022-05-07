import atexit
class A:
    def __init__(self, param):
        self.param = param
        atexit.register(self.close)
    
    def close(self):
        print("closing A param {}".format(self.param))


# def main():
#     try : 
#         driver = Driver()
#         # Run driver. This will block
#         driver.run()
#     except KeyboardInterrupt:     
#         all_stop()


if __name__ == "__main__":
    print("Started foo")
    a = A("foo")
    b = A("bar")
    for x, y  in list(zip(a.__dict__.values(), b.__dict__.values())):
        print(x, y )
    # time.sleep(10)
    #exit()

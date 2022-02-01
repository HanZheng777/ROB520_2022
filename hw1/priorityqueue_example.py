from queue import PriorityQueue

#defines a basic node class
class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in

    def printme(self):
        print("\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid)

if __name__ == "__main__":
    #initialize the priority queue
    q = PriorityQueue()

    #insert objects of form (priority, Node)
    # This will cause error when executing line 24.
    # The new behavior tries to compare the second element when the first element is the same.
    # q.put((1.46, Node(2,3,0.3,1,0)))
    # q.put((2.6, Node(5,2,0.1,2,1)))
    # q.put((5.6, Node(2,3,0.3,3,2)))
    # q.put((0.6, Node(4,3,0.2,4,1)))
    # q.put((0.6, Node(5,2,0.6,4,1)))

    # solution: assign a comparable unique ID to every tuple
    q.put((-1, 2.6, 3))
    q.put((-1, 1.5, 3))
    q.put((-1, 1.7, 3))
    q.put((-2, 1.4, 2))
    q.put((-2, 0.33, 1))
    q.put((-2, 1.5, 2))
    q.put((-2, 0.67, 1))
    # q.put((5.6, 2, Node(2,3,0.3,3,2)))
    # q.put((0.6, 3, Node(4,3,0.2,4,1)))
    # q.put((0.6, 4, Node(5,2,0.6,4,1)))

    # print("Pop elements of queue in order of increasing priority:")
    # next_item = q.get()
    # print("Priority:", next_item[0])
    # next_item[2].printme()
    # next_item = q.queue
    # print(next_item)
    # c = q.queue[1]
    # # print(c[2]==3)
    # print(any(1.2 in item for item in q.queue))
    # for item in q.queue:
    #     if item[2] == 1:
    #         print(True)
    # for item in q.queue:
    #     if Node(2,3,0.3,1,0) in item[2]:
    #         print("true")

    a=q.get((-1, 1.5, 3))
    print(a)
    # while not q.empty():
    #     next_item = q.get()
    #     print("Priority:", next_item[0])
    #     print("f:", next_item[1])

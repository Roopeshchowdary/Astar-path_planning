import numpy as np
import cv2

img=cv2.imread('1-2.png',1)

img1=img.copy()

w, h, c = img.shape

class Node():
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        self.g = np.inf
        self.h = np.inf
        self.f = np.inf

def get_min_dist_node(open_list):                           #finds node with least expected total distance among array elements 
    min_dist = np.inf
    min_node = None
    for node in open_list:
        if open_list[node].f < min_dist:
            min_dist = open_list[node].f
            min_node = open_list[node]
    return min_node

def get_dist(p1, p2):                                          #calculates length of straight line between two nodes
    x1, y1 = p1
    x2, y2 = p2
    return (((x1-x2)**2 + (y1-y2)**2))**0.5

def show_path(node):
    path = []
    while node.parent is not None:
        path.append(node.position)
        cv2.line(img1, node.position,node.parent.position , (255,255,0), 2)         #creates a line
        node = node.parent
        
    cv2.namedWindow('final path', cv2.WINDOW_NORMAL)
    cv2.imshow("final path", img1)                                         #shows final path
    cv2.imwrite("Astar.png",img1)
    if cv2.waitKey(1) == 'q':
        cv2.destroyAllWindows()
        return

def astar(start, end):
    
    open_list = {}
    closed_list = {}
    start_node =  Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    open_list[start] = start_node
    closed_list[start]=Node(None,start)
    while len(open_list)>0:
        current_node = get_min_dist_node(open_list)             #gives priority to node with least expected total distance
        
        open_list.pop(current_node.position)
        
        if current_node.position == end:
            show_path(current_node)
            return
        img[current_node.position[1]][current_node.position[0]] = (0,255,0)
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                
                if ((node_position[0]<=(h-1) and node_position[0]>=0 and node_position[1]<=(w-1) and node_position[1]>=0)):
                    if (img[node_position[1],node_position[0]]==255).all() or (node_position==(359,563)):
                                                                                                            #checks if node crosses allowed limits
                        new_node = Node(current_node, node_position)
                        img[node_position[1]][node_position[0]]=(0,0,254)
                        new_node.g = current_node.g + get_dist(current_node.position, new_node.position)
                        new_node.h = get_dist(new_node.position, end)
                        new_node.f = new_node.g + new_node.h

                        if not(img[new_node.position[1],new_node.position[0]]==(0,255,0)).all(): #checks if node has already been visited
                            closed_list[new_node.position]=new_node                               
                            open_list[new_node.position]=new_node
                        
        cv2.namedWindow('path finding',cv2.WINDOW_NORMAL)
        cv2.imshow('path finding',img)
        cv2.waitKey(1)

start = (880,145)
end = (359,563)

astar(start, end)

cv2.waitKey(0)
cv2.destroyAllWindows()        
               

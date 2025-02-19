# Bruce Bearden
# EENG 350 Assingment 1

import numpy as np
from collections import Counter

# part 1
with open('datafile.txt','r') as f:
    b = eval(f.read())

#return max
print("The max of the list is:", max(b))

#return min
print("The min of the list is:", min(b))

#return index of 38
print("The index of 38 is:", b.index(3))

#numbers(s) that are repeated the most and their occurances
counter = Counter(b)
mostCommon = counter.most_common()
print(mostCommon)

#sorted list (convert to numpy)
bNumpy = np.array(b)
bNumpySorted = np.sort(bNumpy)
print("Sorted list:", bNumpySorted)

#even numbers in order
evenNumsArr = list(filter(lambda x: x%2 == 0, b)) # [1]
print("Even number list in order of appearance:", evenNumsArr)

# part 2

# user inputs a string
stringIn = input("Input a string of characters:")
print(stringIn)
currIdx = 0
print(stringIn[0])
# define state variable, and initialize state 0
state = 0
# create a dictionary to describe the states
state_dictionary = {
    0 : "start",
    1 : "a",
    2 : "b",
    3 : "c",
    4 : "d"
    }
# function for state 0
def state0(activity):
# you can put any code here that should run in state 0
# athough this example doesn’t have any
#
# At the end, we decide what the next state should be
    if activity == "push":
        print("turnstile was pushed, turnstile remaining locked")
        return 0
    elif activity == "coin":
        print("coin was entered, turnstile becoming unlocked")
        return 1
    else:
        return None
# function for state 1
def state1(activity):
    if activity == "push":
        print("turstile was pushed, turnstile becoming locked")
        return 0
    elif activity == "coin":
        print("coin was entered, turnstile remaining unlocked")
        return 1
    else:
        return None
# initalization
state = 0 # initial state
print("Type coin or push to run state machine")
print("Anything else will end the program")
# main loop
while state is not None: # Run until state is None
    print("Current state is "+state_dictionary[state])
    # Get current activity
    activity = input("input: ")
    if state==0:
        state=state0(activity);
    elif state==1:
        state=state1(activity);
    else:
        Exception("Invalid state has been entered")
print("Done with state machine")
# references
# [1] https://stackoverflow.com/questions/46492379/list-comprehension-without-for

import sys
import math


def main():
    filename = sys.argv[1]
    infile = open(filename, 'r')
    numbers = []
    for line in infile:
        if (not line.startswith('\n')):
            numbers.append(float(line))

    total = 0.0
    for num in numbers:
        total += num
    average = total / len(numbers)

    total = 0.0
    for num in numbers:
        total += (num - average)**2
    std_dev = math.sqrt(total / len(numbers))

    # print the results
    print('Average: %.2f' % average)
    print('Standard deviation: %.2f' % std_dev)

main()
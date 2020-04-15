import re

class Optimizer:
    def __init__(self):
        self.filePath = "./data.txt"
        self.file = self.read_in()
        self.data = []
        if self.doesFileExist():
            temp = self.file.readlines()
            for l in temp:
                self.data.append(l.strip('\n'))

    def read_in(self):
        try:
            file = open(self.filePath, "r")
            pass
        except:
            file = 0
        return file

    def doesFileExist(self):
        return not self.file == 0

    def addData(self, data):
        self.data.append(data)

    def checkPoints(self, start, end):
        pass

    def writeData(self):
        with open(self.filePath, "w") as f:
            for d in self.data:
                f.write("%s\n" % d)

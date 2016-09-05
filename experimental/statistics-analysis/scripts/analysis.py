import matplotlib.pyplot as plt
import numpy as np
import os

class TriangulationData:
  def __init__(self, foldername):
    self.foldername = foldername
    self.grid = readgridfile(foldername + "/grid.txt")
    self.info = readinfofile(foldername + "/info.txt")

  def get(self, i, j, k):
    return self.grid[i][j][k]

  def get_param(self,param):
    return self.info[param]

class CummulativeData:
  def __init__(self, rootfolder):
    self.rootfolder = rootfolder
    self.all_data_dirs = [x[1] for x in os.walk(rootfolder)][0]
    self.all_data = [TriangulationData(rootfolder+"/"+x) for x in self.all_data_dirs]

  def get_all(self,i,j,k):
    return [x.get(i,j,k) for x in self.all_data]

  def make_histograms(self, dim):
    fig = plt.figure()
    for i in xrange(5):
      for j in xrange(5):
        sp = fig.add_subplot(5,5,5*i +j+1)
        sp.hist(self.get_all(i,j,dim))
    fig.show()
    # x=raw_input()

  ##### k is x or y or z value of the graph and i and j are the coordinates of the points 
  def make_plot(self,i,j,k,param):
    grid_points = self.get_all(i,j,k)
    param_plot = [x.get_param(param) for x in self.all_data]  
    plot_pair = zip(param_plot,grid_points)
    plot_pair.sort(key=lambda x:x[0])
    plot_pair = zip(*plot_pair)
    plt.plot(plot_pair[0],plot_pair[1])
    plt.show()
    y = raw_input()

def readgridfile(filename):
  f = open(filename,'r')
  lines = f.read().split('\n')[:-1]
  grid = []
  for line in lines:
    gridline = []
    splitted = line.split(';')
    for points in splitted[:-1]:
      points_split = map(float,points[1:-1].split(','))
      gridline.append(points_split)
    grid.append(gridline)
  return grid

def readinfofile(filename):
  f = open(filename, 'r')
  lines =f.read().split('\n')[:-1]
  info = {}
  # gridsize
  info["gridsize"] = map(int,lines[0].split())
  info["angle"] = int(lines[1])
  info["motion"] = int(lines[2])
  info["distance"] = float(lines[3])
  info["intrinsics"] = map(float, lines[4].split())
  info["starting_point"] = map(float, lines[5][1:-1].split(','))
  return info

def main():
  cumdata = CummulativeData("data")
  # print cumdata.get_all(3,4,0)
  # cumdata.make_histograms(0)
  cumdata.make_plot(0,0,2,"distance")
  # print cumdata.all_data_dirs
  # file1 = TriangulationData("tempdir")
  # print file1.grid
  # print file1.foldername
  # print file1.info

if __name__ == '__main__':
  main()

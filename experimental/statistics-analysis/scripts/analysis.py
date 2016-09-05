import matplotlib.pyplot as plt
import numpy as np
import os
import math

class TriangulationData:
  def __init__(self, foldername):
    self.foldername = foldername
    self.grid = readgridfile(foldername + "/grid.txt")
    self.info = readinfofile(foldername + "/info.txt")

  def get(self, i, j, k):
    return self.grid[i][j][k]

  def get_param(self,param):
    return self.info[param]

  def get_avg(self, k):
    ans = 0
    for i in xrange(5):
      for j in xrange(5):
        ans += self.grid[i][j][k]
    return ans/25.0

  def get_median(self,k):
    values = []
    for i in xrange(5):
      for j in xrange(5):
        values.append(self.grid[i][j][k])
    values = np.array(values)
    return np.median(values)

  def get_variance(self,k):
    mean = self.get_avg(k)
    ans = 0
    for i in xrange(5):
      for j in xrange(5):
        ans += math.pow(self.grid[i][j][k] - mean,2)
    ans = ans/25.0 
    return ans

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
    # y = raw_input()

  def plot_avg(self, k, param):
    param_plot = [x.get_param(param) for x in self.all_data] 
    avg_list = [x.get_avg(k) for x in self.all_data]
    median_list = [x.get_median(k) for x in self.all_data]
    var_list = [x.get_variance(k) for x in self.all_data]    
    
    avg_pair = zip(param_plot, avg_list)
    median_pair = zip(param_plot, median_list)
    var_pair = zip(param_plot, var_list)
    
    avg_pair.sort(key = lambda x:x[0])
    median_pair.sort(key = lambda x:x[0])
    var_pair.sort(key = lambda x:x[0])

    avg_pair = zip(*avg_pair)
    # print avg_pair
    median_pair = zip(*median_pair)
    var_pair = zip(*var_pair)

    plt.figure(1)
    plt.subplot(311)
    plt.plot(avg_pair[0],avg_pair[1])

    plt.subplot(312)
    plt.plot(median_pair[0], median_pair[1])

    plt.subplot(313)
    plt.plot(var_pair[0], var_pair[1])

    plt.show()
    z = raw_input()

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
  info["num_images"] = int(lines[6])
  return info

def main():
  cumdata = CummulativeData("data")
  # print cumdata.get_all(3,4,0)
  # cumdata.make_histograms(0)
  # cumdata.make_plot(0,0,2,"distance")

  cumdata.plot_avg(2,"distance")
  # print cumdata.all_data_dirs
  # file1 = TriangulationData("tempdir")
  # print file1.grid
  # print file1.foldername
  # print file1.info

if __name__ == '__main__':
  main()

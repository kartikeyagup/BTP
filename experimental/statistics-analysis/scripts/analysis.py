import matplotlib as plt
import numpy as np

class TriangulationData:
  def __init__(self, foldername):
    self.foldername = foldername
    self.grid = readgridfile(foldername + "/grid.txt")
    self.info = readinfofile(foldername + "/info.txt")

class CummulativeData:
  def __init__(self, rootfolder):
    self.rootfolder = rootfolder
    # TODO: Iterate in all directories and process
    self.all_data = []

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
  info["distance"] = int(lines[3])
  info["intrinsics"] = map(float, lines[4].split())
  info["starting_point"] = map(float, lines[5][1:-1].split(','))
  return info

def main():
  file1 = TriangulationData("tempdir")
  print file1.grid
  print file1.foldername
  print file1.info

if __name__ == '__main__':
  main()

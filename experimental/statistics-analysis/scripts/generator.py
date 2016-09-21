import os
import itertools
import threading

def main():
  num_threads=8
  distances = [i for i in xrange(100,200,100)]
  angles = [i for i in xrange(0,1,1)]
  starting_points_z = [i for i in xrange(300,5000,50)]
  num_images=[2]
  dump_images=[1]
  verbose=[0]
  gridx=[300]
  gridy=[200]
  motion = [1]
  allpossibilites = itertools.product(distances,angles,starting_points_z,num_images,dump_images,verbose,motion,gridx,gridy)
  params = list(allpossibilites)
  chunksize = len(params)/num_threads
  threads=[]
  for i in xrange(num_threads):
    sublist = params[i*chunksize:min((1+i)*chunksize,len(params))]
    t = threading.Thread(target=simulateAll,args=(sublist,))
    threads.append(t)
  for t in threads:
    t.start()
  for t in threads:
    t.join()

def simulateAll(listParams):
  for param in listParams:
    simulateData(param)

def simulateData((distance,angle,starting_point,num_images,dump_images,verbose,motion,gridx,gridy)):
  dirname = "data/"+"_".join(["d",str(distance),"a",str(angle),"stz",str(starting_point),"n",str(num_images),"m",str(motion)])
  command = "build/sensitivity"
  command += " --distance="+str(distance)
  command += " --dirname="+dirname
  command += " --angle="+str(angle)
  command += " --starting_z="+str(starting_point)
  command += " --num_images="+str(num_images)
  command += " --dump_images="+str(dump_images)
  command += " --verbose="+str(verbose)    
  command += " --motion="+str(motion)
  command += " --gridx="+str(gridx)
  command += " --gridy="+str(gridy)
  os.system(command)

if __name__ == '__main__':
  main()

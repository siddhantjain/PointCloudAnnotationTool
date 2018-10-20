# Extrapolate the number of points to 2048
# h5 file has data (1x2048x3), label (1x1) and seg (1x2048)
import h5py
import numpy as np
import sys
import argparse
import os

#siddhant: May not need this. Confirm and remove
def pc_augment_to_point_num(pts, pn):
    assert(pts.shape[0] <= pn)
    cur_len = pts.shape[0]
    res = np.array(pts)
    while cur_len < pn:
        res = np.concatenate((res, pts))
        cur_len += pts.shape[0]
    return res[:pn, :]

#siddhant: May not need this. Confirm and remove
def pc_normalize(pc):
    l = pc.shape[0]
    centroid = np.mean(pc, axis=0)
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc**2, axis=1)))
    pc = pc / m
    return pc

def convertToH5(fileName,numClasses):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    text_file = open(fileName, "r")
    input = text_file.read().split('\n')
    l = []
    for i in range(len(input)):
        x = list(input[i].split())
        l.append(x)
    l = l[:-1]
    points = np.array(l, dtype=np.float)
    point_data = points[:,0:3]
    point_data = np.expand_dims(point_data, axis=0)
    point_seg = points[:, 3]

    point_seg = np.expand_dims(point_seg, axis=0)

    #Siddhant: Explore if point label plays a major role. So far
    # we have worked only with chairs and this is supposedly the class for chair
    # Try with other classes and see if that makes a difference.
    # if it does, we need to pass that as well (which makes things a bit tricky)
    point_label = 5
    point_label = np.expand_dims(point_label, axis=0)
    point_label = np.expand_dims(point_label, axis=0)


    #siddhant: look up the API and store the h5 file in the same folder as the input point cloud file
    h5_filename = os.path.dirname(fileName) + "/" + os.path.basename(fileName).split(".")[0] + '.h5'
    hf = h5py.File(h5_filename, 'w')
    hf.create_dataset('data', data=point_data)
    hf.create_dataset('label', data=point_label)
    hf.create_dataset('pid', data=point_seg)
    hf.close()


    f = h5py.File(h5_filename)
    print(h5_filename)
    data = f['data'][:]
    label = f['label'][:]
    seg = f['pid'][:]

def parse_arguments():
    parser = argparse.ArgumentParser(description='Convert Point Cloud to H5 Parser')
    parser.add_argument('--filename',dest='fname',type=str)
    parser.add_argument('--numclasses',dest='numclasses',type=int,default=0)

    return parser.parse_args()

def main(args):
    args = parse_arguments()
    fileName = args.fname
    numClass = args.numclasses

    convertToH5(fileName,numClass)

if __name__ == '__main__':
    main(sys.argv)
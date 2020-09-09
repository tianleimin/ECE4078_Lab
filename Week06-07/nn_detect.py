# visualize the trained neural network on a test image
# simply run this script to visualize the prediction, no need to change anything
# this visualisation may help you decide how to tune your neural network

import time

import torch
import torch.nn as nn
from PIL import Image
from matplotlib import pyplot as plt
from torchvision import transforms
import matplotlib.patches as label_box

import numpy as np
from torch.utils.data import Dataset, DataLoader

from nn_config import NNState


class Evaluate:
    def __init__(self):
        self.nn_state = NNState(mode='eval')

    def sliding_window(self, img):
        """
        This function converts the classifier has been trained to a detector
        :param img: Input image in the format of PIL. Use Image.open(image_path)
        to read the image.
        :return: a single-channel heat map. with labels (1, 2, 3, ...)
        """
        img = np.array(img)
        w, h, _ = img.shape
        start_time = time.time()
        # the step size of moving the sliding window, measured in pixels
        stride = 8
        # Generate a grid of centers for the window
        u_mesh, v_mesh = np.meshgrid(np.arange(h, step=stride),
                                 np.arange(w, step=stride))
        # the height, and width of the output heat map
        h_out = len(np.arange(h, step=stride))
        w_out = len(np.arange(w, step=stride))
        u_mesh, v_mesh = u_mesh.reshape(-1), v_mesh.reshape(-1)
        print('\n Generating Anchors ...')
        all_anchors = list()
        # number of crops at each designated window waitpoint
        anchor_h2ws = list([0.6, 1, 1.5])  # different height to width ratio
        anchor_heights = list([32, 64])  # the height of the sliding window
        num_patches = 6
        num_anchors = len(anchor_h2ws)*len(anchor_heights)
        for i in range(len(u_mesh)):
            uv = [u_mesh[i], v_mesh[i]]
            anchors_temp = self.get_multi_scal_anchors(uv, img,
                                                       anchor_h2ws,
                                                       anchor_heights)
            all_anchors += anchors_temp
        anchor_imdb = AnchorIMDB(all_anchors)
        anchor_loader = DataLoader(anchor_imdb,
                                   batch_size=num_patches*num_anchors,
                                   shuffle=False, num_workers=4,
                                   drop_last=False)
        heat_map = list()
        print('\n Inferring ...')
        with torch.no_grad():
            sigmoid = nn.Sigmoid()
            for batch in anchor_loader:
                batch = self.nn_state.to_device(batch)
                x = self.nn_state.net(batch)
                # HERE DEFINES THE LOGIC OF CHOOSING THE FINAL LABEL OUT OF
                #   N ANCHORS
                x = sigmoid(x).reshape((num_patches, num_anchors, -1))
                # for heat map of all segments
                val, _ = torch.max(x, 1)
                score, pred = torch.max(val, 1)
                heat_map += pred.data.reshape(num_patches)
        print("--- %.3fs seconds ---" % (time.time() - start_time))
        # print("Prediction of the image label: ", pred1)
        heat_map = np.asarray(heat_map).reshape(w_out, h_out)
        return heat_map

    def visualise_heatmap(self, heat_map, img, overlay=True):
        """
        This function visualises the heat_map
        :param heat_map:
        :param img:
        :param overlay: True to display the mask on top of the image. False to
         display separately.
        """
        h, w = heat_map.shape
        out = np.ones((h, w, 3))
        sheep = np.array([245, 114, 66])/255.0
        coke = np.array([80, 80, 80])/255.0
        neither = np.array([66, 135, 245])/255.0
        for i in range(h):
            for j in range(w):
                if heat_map[i, j] == 0:
                    out[i, j, :] *= coke
                elif heat_map[i, j] == 1:
                    out[i, j, :] = neither
                elif heat_map[i, j] == 2:
                    out[i, j, :] = sheep
        sheep_label = label_box.Patch(color=sheep, label='sheep[2]')
        coke_label = label_box.Patch(color=coke, label='coke[0]')
        neither_label = label_box.Patch(color=neither, label='neither[1]')
        if overlay:
            out = Image.fromarray((out*255).astype('uint8'))
            out = out.resize(img.size)
            out = out.convert("RGBA")
            img = img.convert("RGBA")
            out = Image.blend(img, out, alpha=.6)
            plt.legend(handles=[sheep_label, coke_label, neither_label])
            plt.imshow(out)
        else:
            fig, ax = plt.subplots(1, 2)
            ax[1].legend(handles=[sheep_label, coke_label, neither_label])
            ax[0].imshow(img)
            ax[1].imshow(out)
        plt.show()

    def get_multi_scal_anchors(self, uv, np_img, anchor_h2ws, anchor_heights):
        """
        Crops the image into sizes of the anchor boxes
        :param uv: the window centre location
        :param np_img: the original PIL image
        :param anchor_h2ws: the height to width ratio of anchor boxes
        :param anchor_heights: the height of the anchor bo
        :return:
        """
        h_max, w_max, _ = np_img.shape
        u, v = uv[0], uv[1]
        img_batch = list()
        for h in anchor_heights:
            for h2w in anchor_h2ws:
                win_size = np.array([h, int(h/h2w)])
                half_win = (win_size/2.0).astype(int)
                v_min = max(0, v - half_win[1])
                v_max = min(h_max, v + half_win[1])
                u_min = max(0, u - half_win[0])
                u_max = min(w_max, u + half_win[0])
                anchor_temp = np_img[v_min: v_max, u_min: u_max, :]
                anchor_temp = Image.fromarray(anchor_temp)
                img_batch.append(anchor_temp)
        return img_batch


class AnchorIMDB(Dataset):

    def __init__(self, img_stack):
        self.img_stack = img_stack
        self.img_transform = transforms.Compose(
            [transforms.Resize([64, 64]),
             transforms.ToTensor(),
             transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                  std=[0.229, 0.224, 0.225])])

    def __len__(self):
        return len(self.img_stack)

    def __getitem__(self, idx):
        img = self.img_stack[idx]
        sample = self.img_transform(img)
        return sample


if __name__ == '__main__':
    exp = Evaluate()
    img_path = './test.png'
    img = Image.open(img_path)
    heat_map = exp.sliding_window(img)
    exp.visualise_heatmap(heat_map, img)

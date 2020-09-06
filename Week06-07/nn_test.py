# test the pre-trained neural network on a given test set
# simly run this script to test the network, no need to change anything
# you should only test the neural network on the test set once to evaluate its robustness

import time

import torch
import torch.nn as nn
from PIL import Image
from matplotlib import pyplot as plt
from torchvision import transforms, datasets
import matplotlib.patches as label_box

import numpy as np
from torch.utils.data import Dataset, DataLoader

from nn_config import NNState


class Test:
    def __init__(self):
        self.net_dict = NNState(mode='eval')
        # Data Augmentation operations
        img_transforms = transforms.Compose(
            [transforms.RandomRotation((-30, 30)),
             transforms.RandomResizedCrop((64, 64), scale=(0.7, 1.0)),
             transforms.ColorJitter(brightness=0.4, contrast=0.3,
                                    saturation=0.3, hue=0.3),
             transforms.ToTensor(),
             transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                  std=[0.229, 0.224, 0.225])])
        self.eval_data = datasets.ImageFolder('./dataset_segmented/test',
                                              transform=img_transforms)


    def eval(self):
        print('Evaluating...')
        self.net_dict.net = self.net_dict.net.eval()
        eval_loader = DataLoader(dataset=self.eval_data,
                                 batch_size=self.net_dict.batch_size,
                                 shuffle=False, num_workers=0,
                                 drop_last=False)
        n_batch = len(eval_loader)
        with torch.no_grad():
            eval_loss_stack = self.net_dict.to_device(torch.Tensor())
            correct = 0
            total = 0
            for i, batch in enumerate(eval_loader):
                # forward propagation
                inputs, labels = batch[0], batch[1]
                inputs = self.net_dict.to_device(inputs)
                labels = self.net_dict.to_device(labels)
                # Forward
                labels_hat = self.net_dict.net.forward(inputs)
                _, predicted = torch.max(labels_hat.data, 1)
                total += labels.size(0)
                correct += (predicted == labels).sum().item()
                loss_batch = self.net_dict.criterion(labels_hat, labels)
                eval_loss_stack = torch.cat(
                    (eval_loss_stack, loss_batch.unsqueeze(0)), 0)
                print('Batch [%d/%d], Eval Loss: %.4f'
                      % (i + 1, n_batch, loss_batch))
            eval_loss = torch.mean(eval_loss_stack)
            print('*********************************')
            print('=> Mean Evaluation Loss: %.3f' % eval_loss)
            print('=> Accuracy of the network: %d %%' % (
                    100 * correct / total))
            print('*********************************')
        return eval_loss



if __name__ == '__main__':
    exp = Test()
    exp.eval()

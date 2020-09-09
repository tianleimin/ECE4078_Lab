# training the neural network on the training set and evaluate it on the evaluation set
# simly run this script to train the network, no need to change anything

# the training parameters, which you may tune, are specified in nn_config.yml
# the model structure, which you may optimize, is specified in net_model/alex_net.py
# tune your neural network on the evaluation set, NOT on the test set
# you should only test the neural network on the test set once to evaluate its robustness

# the best performing model will be saved in net_weights/exp_name
# this pretrained model will be used by nn_test.py and nn_detect.py

import time
import torch
from torchvision import transforms, datasets
from torch.utils.data import DataLoader

from nn_config import NNState


class Train:
    def __init__(self):
        self.net_dict = NNState('train')
        # Data Augmentation operations
        img_transforms = transforms.Compose(
            [transforms.RandomRotation((-30, 30)),
             transforms.RandomResizedCrop((64, 64), scale=(0.7, 1.0)),
             transforms.ColorJitter(brightness=0.4, contrast=0.3,
                                    saturation=0.3, hue=0.3),
             transforms.ToTensor(),
             transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                  std=[0.229, 0.224, 0.225])])

        self.train_data = datasets.ImageFolder('./dataset_segmented/train',
                                               transform=img_transforms)
        print(self.train_data.class_to_idx)
        self.eval_data = datasets.ImageFolder('./dataset_segmented/eval',
                                              transform=img_transforms)

    def train(self):
        train_loader = DataLoader(dataset=self.train_data,
                                  batch_size=self.net_dict.batch_size,
                                  shuffle=True, num_workers=4,
                                  drop_last=True)
        n_batch = len(train_loader)
        for epoch_idx in range(self.net_dict.last_epoch + 1,
                               self.net_dict.n_epochs):
            train_loss_buff = torch.Tensor()
            train_loss_buff = self.net_dict.to_device(train_loss_buff)
            print('\nEpoch [%d/%d]:' % (epoch_idx, self.net_dict.n_epochs))
            t_start = time.time()
            # update the network
            for i, batch in enumerate(train_loader):
                self.net_dict.optimiser.zero_grad()
                inputs, labels = batch[0], batch[1]
                inputs = self.net_dict.to_device(inputs)
                labels = self.net_dict.to_device(labels)
                # Forward
                labels_hat = self.net_dict.net.forward(inputs)
                loss = self.net_dict.criterion(labels_hat, labels)
                # Backward
                loss.backward()
                # Optimise
                self.net_dict.optimiser.step()
                train_loss_buff = torch.cat((train_loss_buff,
                                             loss.reshape(1, 1)), 0)
                if (i + 1) % 10 == 0:
                    print('[%d/%d], Itr [%d/%d], Loss: %.4f'
                          % (epoch_idx, self.net_dict.n_epochs, i,
                             n_batch, loss.item()))
            # current_lr = self.optimiser.param_groups[0]['lr']
            self.net_dict.lr_scheduler.step()
            avg_train_loss = torch.mean(train_loss_buff)
            print('=> Average training loss: %.4f' % avg_train_loss)
            print('Training Duration: %.3fs' % (time.time() - t_start))
            if (epoch_idx+1) % 1 == 0:
                eval_loss_mean = self.eval()
                # Save model, and best model if qualified
                delta_acc = self.net_dict.best_acc - eval_loss_mean
                if delta_acc > 0:
                    self.net_dict.best_acc = eval_loss_mean
                self.net_dict.save_ckpt(epoch_idx, delta_acc)

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
    torch.manual_seed(1)
    exp = Train()
    exp.train()

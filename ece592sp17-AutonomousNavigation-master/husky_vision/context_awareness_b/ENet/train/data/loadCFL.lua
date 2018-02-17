----------------------------------------------------------------------
-- Cityscape data loader,
-- Abhishek Chaurasia,
-- February 2016
----------------------------------------------------------------------

require 'torch'   -- torch
require 'image'   -- to visualize the dataset

torch.setdefaulttensortype('torch.FloatTensor')
----------------------------------------------------------------------
-- Cityscape dataset:

local trsize, tesize

trsize = 790 -- cityscape train images
tesize = 80  -- cityscape validation images
local classes = {'Unlabeled', 'Road', 'object', 'person', 'sky'}
local conClasses = {'Road', 'object', 'person', 'sky'}

local nClasses = #classes

--------------------------------------------------------------------------------

-- Ignoring unnecessary classes
print '==> remapping classes'
local classMap = {[0] =  {1}, -- Unlabeled
                  [1]  =  {2}, -- road
                  [100]  =  {3}, -- object
                  [150]  =  {4}, -- person
                  [200]  =  {5}, -- sky
                              }

-- Reassign the class numbers
local classCount = 1

if opt.smallNet then
classMap = {[0] =  {1}, -- Unlabeled
            [1]  =  {2}, -- road
            [100]  =  {3}, -- object
            [150]  =  {3}, -- person
            [200]  =  {3}, -- sky
                              }

classes = {'Unlabeled', 'road', 'object' }
conClasses = {'road', 'object' } -- 7 classee
end
-- From here #class will give number of classes even after shortening the list
-- nClasses should be used to get number of classes in original list

-- saving training histogram of classes
local histClasses = torch.Tensor(#classes):zero()

print('==> number of classes: ' .. #classes)
print('classes are:')
print(classes)

--------------------------------------------------------------------------------
print '==> loading cfl dataset'
local trainData, testData
local loadedFromCache = false
paths.mkdir(paths.concat(opt.cachepath, 'cfl'))
local cflCachePath = paths.concat(opt.cachepath, 'cfl', 'data.t7')

if opt.cachepath ~= "none" and paths.filep(cflCachePath) then
   local dataCache = torch.load(cflCachePath)
   trainData = dataCache.trainData
   testData = dataCache.testData
   histClasses = dataCache.histClasses
   loadedFromCache = true
   dataCache = nil
   collectgarbage()
else
   local function has_image_extensions(filename)
      local ext = string.lower(path.extension(filename))

      -- compare with list of image extensions
      local img_extensions = {'.jpeg', '.jpg', '.png', '.ppm', '.pgm'}
      for i = 1, #img_extensions do
         if ext == img_extensions[i] then
            return true
         end
      end
      return false
   end

   -- initialize data structures:
   trainData = {
      data = torch.FloatTensor(trsize, opt.channels, opt.imHeight, opt.imWidth),
      labels = torch.FloatTensor(trsize, opt.labelHeight, opt.labelWidth),
      preverror = 1e10, -- a really huge number
      size = function() return trsize end
   }

   testData = {
      data = torch.FloatTensor(tesize, opt.channels, opt.imHeight, opt.imWidth),
      labels = torch.FloatTensor(tesize, opt.labelHeight, opt.labelWidth),
      preverror = 1e10, -- a really huge number
      size = function() return tesize end
   }

   print('==> loading training files');

   local dpathRoot = opt.datapath .. '/RawImages/train/'

   assert(paths.dirp(dpathRoot), 'No training folder found at: ' .. opt.datapath)
   --load training images and labels:
   local c = 1
   for dir in paths.iterdirs(dpathRoot) do
      local dpath = dpathRoot .. dir .. '/'
      for file in paths.iterfiles(dpath) do
         print(file)
         -- process each image
         if has_image_extensions(file) and c <= trsize then
            local imgPath = path.join(dpath, file)

            --load training images:
            local dataTemp = image.load(imgPath)
            trainData.data[c] = image.scale(dataTemp,opt.imWidth, opt.imHeight)

            -- Load training labels:
            -- Load labels with same filename as input image.
            imgPath = string.gsub(imgPath, "RawImages", "Annotations")
            imgPath = string.gsub(imgPath, ".jpg", "_annotated_image.png")


            -- label image data are resized to be [1,nClasses] in [0 255] scale:
            local labelIn = image.load(imgPath, 1, 'byte')
            local labelFile = image.scale(labelIn, opt.labelWidth, opt.labelHeight, 'simple'):float()

            labelFile:apply(function(x) return classMap[x][1] end)

            -- Syntax: histc(data, bins, min, max)
            histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)

            -- convert to int and write to data structure:
            trainData.labels[c] = labelFile

            c = c + 1
            if c % 20 == 0 then
               xlua.progress(c, trsize)
            end
            collectgarbage()
         end
      end
   end
   print('')

   print('==> loading testing files');
   dpathRoot = opt.datapath .. '/RawImages/val/'

   assert(paths.dirp(dpathRoot), 'No testing folder found at: ' .. opt.datapath)
   -- load test images and labels:
   local c = 1
   for dir in paths.iterdirs(dpathRoot) do
      local dpath = dpathRoot .. dir .. '/'
      for file in paths.iterfiles(dpath) do

         -- process each image
         if has_image_extensions(file) and c <= tesize then
            local imgPath = path.join(dpath, file)

            --load training images:
            local dataTemp = image.load(imgPath)
            testData.data[c] = image.scale(dataTemp, opt.imWidth, opt.imHeight)

            -- Load validation labels:
            -- Load labels with same filename as input image.
            imgPath = string.gsub(imgPath, "RawImages", "Annotations")
            imgPath = string.gsub(imgPath, ".jpg", "_annotated_image.png")


            -- load test labels:
            -- label image data are resized to be [1,nClasses] in in [0 255] scale:
            local labelIn = image.load(imgPath, 1, 'byte')
            local labelFile = image.scale(labelIn, opt.labelWidth, opt.labelHeight, 'simple'):float()

            labelFile:apply(function(x) return classMap[x][1] end)

            -- convert to int and write to data structure:
            testData.labels[c] = labelFile

            c = c + 1
            if c % 20 == 0 then
               xlua.progress(c, tesize)
            end
            collectgarbage()
         end
      end
   end
end

if opt.cachepath ~= "none" and not loadedFromCache then
   print('==> saving data to cache: ' .. cflCachePath)
   local dataCache = {
      trainData = trainData,
      testData = testData,
      histClasses = histClasses
   }
   torch.save(cflCachePath, dataCache)
   dataCache = nil
   collectgarbage()
end

----------------------------------------------------------------------
print '==> verify statistics'

-- It's always good practice to verify that data is properly
-- normalized.

for i = 1, opt.channels do
   local trainMean = trainData.data[{ {},i }]:mean()
   local trainStd = trainData.data[{ {},i }]:std()

   local testMean = testData.data[{ {},i }]:mean()
   local testStd = testData.data[{ {},i }]:std()

   print('training data, channel-'.. i ..', mean: ' .. trainMean)
   print('training data, channel-'.. i ..', standard deviation: ' .. trainStd)

   print('test data, channel-'.. i ..', mean: ' .. testMean)
   print('test data, channel-'.. i ..', standard deviation: ' .. testStd)
end

----------------------------------------------------------------------

local classes_td = {[1] = 'classes,targets\n'}
for _,cat in pairs(classes) do
   table.insert(classes_td, cat .. ',1\n')
end

local file = io.open(paths.concat(opt.save, 'categories.txt'), 'w')
file:write(table.concat(classes_td))
file:close()

-- Exports
opt.dataClasses = classes
opt.dataconClasses  = conClasses
opt.datahistClasses = histClasses

return {
   trainData = trainData,
   testData = testData,
   mean = trainMean,
   std = trainStd
}

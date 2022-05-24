function memory = batchSample (memory)
memory.train.index = randperm(min(memory.index,memory.capacity),memory.batch_size);
for i = 1:memory.batch_size
    memory.train.data(:,i) = memory.data(:,memory.train.index(i));
end
end
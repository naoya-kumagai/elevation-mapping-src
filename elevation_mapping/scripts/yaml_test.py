import yaml

d = {'A':'a', 'B':{'C':'c', 'D':'d', 'E':'e'}}

for i in range(10):
    with open('testing.yml', 'a') as f:
        yaml.dump(d, f)
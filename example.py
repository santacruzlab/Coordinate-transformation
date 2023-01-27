from rotation import *

diff = [27,36,70]
start_atlas  = [20,24,24]
target_atlas = [14,16,14]

print('Starting point')
start = reference(diff,start_atlas)

print('\nTarget point')
end   = reference(diff,target_atlas)

print('\n')
theta, phi = rotation_main(start=start,target=end)
print('\n')

S = stereotax(*end)
S.rotate(theta,phi)
S.moveback()

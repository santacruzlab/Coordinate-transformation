from rotation import *

# Define the translation between the atlas coordinate and the stereotax coordinate
# E.g., if (0,0,0) in the atlas corresponds to (28,36,)
diff = [28,36,34.5]
start_atlas  = [20,24,24]
target_atlas = [20,23,19]

print('Starting point')
start = reference(diff,start_atlas)

print('Target point')
end   = reference(diff,target_atlas)

print()
theta, phi = rotation_main(start=start,target=end)
print()

azimuthal(angle=phi)
print()

S = stereotax(*start)

print('Before\t', S.cartesian, S.spherical)
S.rotate(theta,phi)
print('After\t', S.cartesian, S.spherical)


S.moveback()


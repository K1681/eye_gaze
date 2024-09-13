resultant_direction = [0, 0]
temp_direction = [0.1, 0.5]

#The following appends lists.
resultant_direction += temp_direction

#The following adds element wise.
resultant_direction[0] += temp_direction[0]
resultant_direction[1] += temp_direction[1]

print(resultant_direction)

        while True:
            # Move elements from top col
            # print(matrix[col, row, 1])
            while row < rows and matrix[col, row, 1] != blocked_value:
                result.append(matrix[col, row, 0])
                # print(matrix[col, row])
                print('1')
                row += 1
            
            row -= 1
            col += 1
            
            if col >= cols or matrix[col, row, 1] == blocked_value:
                break
            
            # Move elements from rightmost rowumn
            while col < cols and matrix[col, row, 1] != blocked_value:
                result.append(matrix[col, row, 0])
                col += 1
                print('2')
            col -= 1
            row -= 1
            
            if row < 0 or matrix[col, row, 1] == blocked_value:
                break
            
            # Move elements from bottom col
            while row >= 0 and matrix[col, row, 1] != blocked_value:
                result.append(matrix[col, row, 0])
                row -= 1
                print('3')
            row += 1
            col -= 1
            
            if col < 0 or matrix[col, row, 1] == blocked_value:
                break
            
            # Move elements from leftmost rowumn
            while col >= 0 and matrix[col, row, 1] != blocked_value:
                result.append(matrix[col, row, 0])
                col -= 1
                print('4')
            col += 1
            row += 1
            
            if row >= rows or matrix[col, row, 1] == blocked_value:
                break

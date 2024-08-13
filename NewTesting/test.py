from subprocess import check_output

output = None
def run_py_file():
    global output
    output = check_output(["python", "test2.py"])



print("Running Test2.py")
run_py_file()
print(f"Program Exit, got: {output}")
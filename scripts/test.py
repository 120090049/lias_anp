import os
file_number = max([int(f[6:]) for f in os.listdir('.') if f.startswith('record') and f[6:].isdigit()])
new_folder = f"record{file_number + 1}"
os.makedirs(new_folder, exist_ok=True)
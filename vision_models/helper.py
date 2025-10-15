import os
from pathlib import Path
import shutil
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import messagebox

class ImageClassifier:
    def __init__(self):
        self.input_dir = Path('./dataset')
        self.output_base_dir = Path('annoted')  # Keeping your original directory name
        self.class_folders = ['I', 'O', 'T', 'X', 'L', 'None']
        self.stats = {'moved': 0, 'skipped': 0}
        self.move_history = []  # Stack to track moves for undo
        
    def setup_directories(self):
        """Create output directories if they don't exist."""
        self.output_base_dir.mkdir(exist_ok=True)
        for folder in self.class_folders:
            (self.output_base_dir / folder).mkdir(exist_ok=True)

    def setup_gui(self):
        """Initialize the GUI window."""
        self.root = tk.Tk()
        self.root.configure(bg='green')
        self.root.title("Image Classification")
        
        # Add keyboard shortcuts
        self.root.bind('i', lambda e: self.process_input('I'))
        self.root.bind('o', lambda e: self.process_input('O'))
        self.root.bind('t', lambda e: self.process_input('T'))
        self.root.bind('x', lambda e: self.process_input('X'))
        self.root.bind('l', lambda e: self.process_input('L'))
        self.root.bind('n', lambda e: self.process_input('None'))
        self.root.bind('q', lambda e: self.quit_application())
        self.root.bind('u', lambda e: self.undo_last_move())
        self.root.bind('<Right>', lambda e: self.process_input(''))
        
        # Add instruction label
        instructions = """
        Keyboard shortcuts:
        I/O/C/X/L - Move to respective folder
        N - Move to None folder
        Q - Quit application
        """
        tk.Label(self.root, text=instructions, bg='green', fg='white').pack(pady=10)

    def display_image(self, img_path):
        """Display the image in the GUI window."""
        try:
            img = Image.open(str(img_path))
            
            # Check image size
            if img.size[0] < 15 or img.size[1] < 15:
                print(f"Image {img_path.name} too small. Skipping...")
                return False
                
            # Resize if image is too large
            max_size = (800, 600)
            if img.size[0] > max_size[0] or img.size[1] > max_size[1]:
                img.thumbnail(max_size, Image.Resampling.LANCZOS)
            
            photo = ImageTk.PhotoImage(img)
            label = tk.Label(self.root, image=photo, bg='green')
            label.image = photo  # Keep a reference!
            label.pack(pady=10)
            
            # Display image info
            info_text = f"Image: {img_path.name}\nSize: {img.size}"
            tk.Label(self.root, text=info_text, bg='green', fg='white').pack()
            
            return True
            
        except Exception as e:
            print(f"Error displaying image {img_path.name}: {str(e)}")
            return False

    def process_input(self, category):
        """Process user input and move the image."""
        if hasattr(self, 'current_image'):
            # TODO: IF THIS BREAKS REMOVE IT
            if category == '':  # Skip case
                print(f"Skipped {self.current_image.name}")
                self.stats['skipped'] += 1
                self.clear_display()
                self.root.quit()
                return
                

            output_path = self.output_base_dir / category / self.current_image.name
            try:
                # Store the move info for undo
                source_path = self.current_image
                shutil.move(source_path, output_path)
                self.move_history.append((str(source_path), str(output_path)))
                self.stats['moved'] += 1
                print(f"Moved {self.current_image.name} to {category} folder")
            except Exception as e:
                print(f"Error moving file: {str(e)}")
            
            self.clear_display()
            self.root.quit()  # Exit the current mainloop() call
            
    def undo_last_move(self):
        """Undo the last move operation."""
        if self.move_history:
            try:
                source_path, dest_path = self.move_history.pop()
                shutil.move(dest_path, source_path)
                self.stats['moved'] -= 1
                print(f"Undid move: {Path(dest_path).name} back to dataset")
            except Exception as e:
                print(f"Error undoing move: {str(e)}")
                # If error occurs, add the move back to history
                self.move_history.append((source_path, dest_path))

    def clear_display(self):
        """Clear the current image display."""
        for widget in self.root.winfo_children():
            widget.destroy()

    def quit_application(self):
        """Quit the application."""
        self.root.destroy()

    def classify_images(self):
        """Main classification loop."""
        try:
            self.setup_directories()
            self.setup_gui()
            
            count = 0
            for img_path in self.input_dir.glob('*'):
                if img_path.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
                    continue
                    
                count += 1
                print(f"\nProcessing image {count}: {img_path}")
                
                self.current_image = img_path
                if self.display_image(img_path):
                    self.root.mainloop()  # Wait for user input
                else:
                    self.stats['skipped'] += 1
                    
            print("\nFinal Statistics:")
            print(f"Moved: {self.stats['moved']}")
            print(f"Skipped: {self.stats['skipped']}")
            
        except Exception as e:
            print(f"Error: {str(e)}")
            messagebox.showerror("Error", f"An error occurred: {str(e)}")
        finally:
            if hasattr(self, 'root') and self.root.winfo_exists():
                self.root.destroy()

def main():
    classifier = ImageClassifier()
    classifier.classify_images()
    print("\nClassification completed!")

if __name__ == "__main__":
    main()
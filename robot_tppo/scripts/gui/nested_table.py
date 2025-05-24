from tkinter import ttk

class NestedTable(ttk.Treeview):
    def __init__(self, root):
        super().__init__(root)
        self["columns"] = ("value",)
        self.column("#0", width=200, stretch=True)
        self.column("value", width=100, stretch=True)
        self.heading("#0", text="Field")
        self.heading("value", text="Value")
        ttk.Style().configure('Treeview', rowheight=30)
        self.item_ids = {}  # To store all item IDs for quick access
        self.path_map = {}  # To store the path to each item

    def build_tree(self, data_dict, parent=""):
        """Builds/rebuilds the tree from dictionary"""
        self.delete(*self.get_children())  # Clear existing items
        self.item_ids = {}
        self.path_map = {}
        self._add_items(data_dict, parent)

    def _add_items(self, data_dict, parent="", current_path=None):
        """Recursively adds items to treeview"""
        if current_path is None:
            current_path = []
            
        for key, value in data_dict.items():
            item_id = f"{parent}_{key}" if parent else key
            path = current_path + [key]
            
            if isinstance(value, dict):
                self.insert(parent, "end", item_id, text=key, values=(""))
                self.item_ids[item_id] = None
                self.path_map[item_id] = path
                self._add_items(value, item_id, path)
            else:
                self.insert(parent, "end", item_id, text=key, values=(value))
                self.item_ids[item_id] = value
                self.path_map[item_id] = path

    def update_values(self, new_data):
        """Updates treeview with new data using the path map"""
        for item_id in self.item_ids:
            if not self.get_children(item_id):  # Only update leaf nodes
                path = self.path_map[item_id]
                try:
                    current = new_data
                    for part in path:
                        current = current[part]
                    self.set(item_id, "value", current)
                except (KeyError, TypeError) as e:
                    print(f"Error updating {item_id}: {e}")

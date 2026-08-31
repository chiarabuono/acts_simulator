import os
import sys

# -- Path setup --------------------------------------------------------
sys.path.insert(0, os.path.abspath('../../'))
sys.path.insert(0, os.path.abspath('../../acts_simulator'))
sys.path.insert(0, os.path.abspath('../../create_xml'))

# -- Project information ------------------------------------------------
project = 'ACTS Simulator'
copyright = '2026, Chiara Buono'
author = 'Chiara Buono'
release = '0.1'

# -- General configuration ------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',       
    'sphinx.ext.napoleon',      
    'sphinx.ext.viewcode',      
    'sphinx.ext.intersphinx',   
    'myst_parser',              
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

autodoc_mock_imports = [
    'mujoco',
    'tkinter',
    'PIL',
    'pyqtgraph',
    'PySide6',
    'imageio',
]

napoleon_google_docstring = True
napoleon_numpy_docstring = True

# -- Options for HTML output -------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_extra_path = ['documents']
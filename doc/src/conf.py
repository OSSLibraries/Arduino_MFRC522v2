# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
import subprocess

sys.path.append(os.path.abspath('../..'))

# Doxygen
subprocess.call('doxygen Doxyfile', shell=True)

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'RFID_MFRC522v2'
copyright = '2023, GithubCommunity'
author = 'GithubCommunity'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'breathe',
    "exhale",
    'myst_parser',
    'notfound.extension',
    'sphinx.ext.autodoc',
    'sphinx.ext.coverage',
    'sphinx.ext.githubpages',
    'sphinx.ext.ifconfig',
    'sphinx.ext.inheritance_diagram',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
]

templates_path = ['_templates']
exclude_patterns = ['Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

## Markdown ##
source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'literal',
    '.md': 'markdown',
}
myst_enable_extensions = [
    "linkify",
    "smartquotes",
    "strikethrough",
    "tasklist",
]

## C++ CPP ##
# Configuration for the breathe extension
# Which directory to read the Doxygen output from
# breathe_projects = {"ModernDoc":"xml"}
# breathe_default_project = "ModernDoc"



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'alabaster'
html_theme = 'nature'
html_static_path = ['_static']



breathe_projects = {
    'MFRC522': "./_doxygen/xml/"
}
breathe_default_project = 'MFRC522'
breathe_default_members = (
    'members', 
    # 'undoc-members',
)

exhale_args = {
    "containmentFolder": "./lib",
    "doxygenStripFromPath": "..",
    "rootFileName": "index.rst",
    "rootFileTitle": "Library API",
    # "createTreeView":        True,
}


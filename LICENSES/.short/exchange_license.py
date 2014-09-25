#!/usr/bin/env python

import getopt
import os
import os.path
import re
import sys

extensions = dict()
extensions['cpp']   = [".h", ".hpp", ".cpp", ".c", ".C", ".cc", ".hxx", ".cxx"]
extensions['cuda']  = [".cu", ".cuh"]
extensions['py']    = [".py"]
extensions['cmake'] = [".find", ".cmake", ".txt"]

def Usage(progname):
    print >> sys.stderr, "Usage: " + os.path.basename(progname) + " [options] [filename]"
    print >> sys.stderr, "  -C DIRECTORY  target directory (can be specified multiple times)"
    print >> sys.stderr, "  -l LICENSE    specify a new license file"
    print >> sys.stderr, "  -f            insert the license header if none was found"
    print >> sys.stderr, "  -o            overwrite existing license"
    print >> sys.stderr, "  -q            don't print status messages"
    print >> sys.stderr, "  -t            filetype ('cmake', 'cpp', 'py' or 'cuda'). Determines the comment indicator and filename patterns. Default is 'c'"
    print >> sys.stderr, "  --dry-run     only print out which files would be changed (overrides -q)"

begin_license_block = dict()
end_license_block = dict()
begin_license_block['cpp']   = re.compile("// -- BEGIN LICENSE BLOCK --")
end_license_block['cpp']     = re.compile("// -- END LICENSE BLOCK --")

begin_license_block['cuda']  = re.compile("// -- BEGIN LICENSE BLOCK --")
end_license_block['cuda']    = re.compile("// -- END LICENSE BLOCK --")

begin_license_block['py']    = re.compile("# -- BEGIN LICENSE BLOCK --")
end_license_block['py']      = re.compile("# -- END LICENSE BLOCK --")

begin_license_block['cmake'] = re.compile("# -- BEGIN LICENSE BLOCK --")
end_license_block['cmake']   = re.compile("# -- END LICENSE BLOCK --")

extended_license_begin = dict()
extended_license_end = dict()
extended_license_begin['cpp']   = ["// -- BEGIN LICENSE BLOCK ----------------------------------------------\n"]
extended_license_end['cpp']     = ["// -- END LICENSE BLOCK ------------------------------------------------\n","\n"]

extended_license_begin['cuda']  = ["// -- BEGIN LICENSE BLOCK ----------------------------------------------\n"]
extended_license_end['cuda']    = ["// -- END LICENSE BLOCK ------------------------------------------------\n","\n"]

extended_license_begin['py']    = ["# -- BEGIN LICENSE BLOCK ----------------------------------------------\n"]
extended_license_end['py']      = ["# -- END LICENSE BLOCK ------------------------------------------------\n","\n"]

extended_license_begin['cmake'] = ["# -- BEGIN LICENSE BLOCK ----------------------------------------------\n"]
extended_license_end['cmake']   = ["# -- END LICENSE BLOCK ------------------------------------------------\n","\n"]



emacs_line = re.compile("-\\*-.*-\\*-")

def ProcessFile(filename, license_text, insert_if_missing, overwrite, quiet, dry_run, filetype):
    if not quiet:
        sys.stderr.write("Processing file " + filename + ": ")

    infile = open(filename, "r")
    input_content = infile.readlines()
    infile.close()

    output_content = []
    existing_license = []

    license_block = False
    license_block_found = False
    for line in input_content:
        if not license_block:
            if begin_license_block[filetype].search(line):
                if not quiet:
                    sys.stderr.write("found license block ... ")
                license_block = True
                license_block_found = True

            output_content += [line]
        else:
            if end_license_block[filetype].search(line):
                license_block = False
                if len(existing_license) == 0 or overwrite:
                    if not quiet:
                        sys.stderr.write("exchanged")
                    output_content += license_text
                else:
                    if not quiet:
                        sys.stderr.write("existing license kept")
                    output_content += existing_license
                output_content += [line]
            elif license_block:
                existing_license += [line]

    if not license_block_found:
        if not quiet:
            sys.stderr.write("no license block ... ")
        if insert_if_missing:
            if not quiet:
                sys.stderr.write("added")
            extended_license_text = extended_license_begin[filetype] + license_text + extended_license_end[filetype]
            if len(output_content)>0 and emacs_line.search(output_content[0]):
                extended_license_text.insert(0, "\n")
                output_content = [output_content[0]] + extended_license_text + output_content[1:]
            else:
                output_content = extended_license_text + output_content[:]
        elif not quiet:
            sys.stderr.write("ignored")

    if not dry_run:
        outfile = open(filename, "w")
        outfile.writelines(output_content)
        outfile.close()

    if not quiet:
        sys.stderr.write("\n")

def FindFiles(directories, filetype):
    file_list = []

    for directory in directories:
        # Recursively search all files and directories.
        for root, dirs, files in os.walk(directory):
            for f in files:
                abs_f = os.path.join(root,f)
                # Also process symbolic links to directories.
                if os.path.isdir(abs_f):
                    file_list += FindFiles(os.readlink(abs_f))
                elif os.path.splitext(f)[1] in extensions[filetype]:
                    file_list.append(abs_f)

    return file_list

def Main(argv):
    # Set default values.
    files = None
    directories = None
    license_file = None
    insert_if_missing = False
    quiet = False
    overwrite = False
    dry_run = False
    filetype = "cpp"

    # Read the commandline arguments.
    try:
        opts, args = getopt.getopt(argv[1:], "C:l:t:foqh", ["directory=", "license=", "type=", "force", "overwrite", "quiet", "help", "dry-run"])
    except getopt.GetoptError:
        print "getopt error"
        sys.exit(1)

    if len(opts) == 0:
        Usage(argv[0])
        return 0

    for opt, arg in opts:
        if opt in ("-C", "--directory"):
            if directories == None:
                directories = [arg]
            else:
                directories += arg

        elif opt in ("-l", "--license"):
            license_file = arg

        elif opt in ("-f", "--force"):
            insert_if_missing = True

        elif opt in ("-q", "--quiet"):
            quiet = True

        elif opt in ("-o", "--overwrite"):
            overwrite = True

        if opt in ("-t", "--type"):
            if arg in ["cmake", "cpp", "cuda", "py"]:
                filetype = arg
            else:
                print "Error: not a known filetype. Use 'cmake', 'cpp', 'cuda', or 'py'!"
                return 0

        elif opt in ("-h", "--help"):
            Usage(argv[0])
            return 0

        elif opt in ("--dry-run"):
            dry_run = True
            quiet = False

    # Set the default directory list if none were specified on the commandline.
    if directories == None:
        directories = [os.path.join(os.environ["MCAHOME"],"libraries"),
                       os.path.join(os.environ["MCAHOME"],"projects"),
                       os.path.join(os.environ["MCAHOME"],"tools"),
                       os.path.join(os.environ["MCAHOME"],"packages")]

    # Check if files were specified on the commandline.
    if len(args) > 0:
        files = args
    else:
        files = FindFiles(directories, filetype)

    # Read the license file.
    license_text = []
    if license_file != None and os.path.exists(license_file):
        infile = open(license_file, "r")
        license_text = infile.readlines()
        infile.close()

    # Process files.
    for filename in files:
        ProcessFile(filename, license_text, insert_if_missing, overwrite, quiet, dry_run, filetype)

    return 0

if __name__ == "__main__":
    sys.exit(Main(sys.argv))

#!/usr/bin/perl

use Getopt::Long;
use File::Basename;
use Cwd 'abs_path', 'getcwd';

sub do_linux () {
  my $fname = "rba-$param_version-src";
   $cdir = getcwd();

  chdir($param_dir) 
    or die "$param_dir: $!";

  `mkdir "$fname"`;
  
  `cp -r "$spath/src" "$fname/"`;
  `cp -r "$spath/mrba" "$fname/"`;
  `cp -r "$spath/doc" "$fname/"`;
  `cp "$spath/CMakeLists.txt" "$fname/"`;

  `tar -jcf "$fname.tar.bz2" "$fname"`;

  `rm -rf "$fname"`;

  chdir($cdir) 
    or die "$cdir: $!";
}

sub do_windows () {
  my $fname = "rba-$param_version-win64";
   $cdir = getcwd();

  chdir($param_dir) 
    or die "$param_dir: $!";

  `mkdir "$fname"`;
  
  `cp -r "$spath/win/usr/bin" "$fname/"`;
  `cp -r "$spath/mrba" "$fname/"`;
  `cp -r "$spath/doc" "$fname/"`;

  `zip -qr "$fname.zip" "$fname"`;

  `rm -rf "$fname"`;

  chdir($cdir) 
    or die "$cdir: $!";
}

###############################################################################
## main ###

GetOptions(
    "version=s"        => \$param_version,
    "os=s"	       => \$param_os,
    "dir=s"	       => \$param_dir,
);

$spath = dirname(abs_path($0));

(defined $param_os)
  or die("Mandatory parameter 'os' not set [linux|win]");

if (not defined $param_dir)
  {
    $param_dir = getcwd();
  }

if (not defined $param_version)
  {
    my $fname = 'src/definitions.h';
    open(my $fh, '<', $fname)
      or die "Could not open file '$fname' $!";

    while(<$fh>)
      {
	if ($_ =~ /RBA_VERSION.*"(.*)".*/)
          {
            $param_version = $1;
          }
      }

    close($fh);
  }

if ($param_os eq 'linux')
  {
    do_linux();
  }
elsif ($param_os eq 'win')
  {
    do_windows();
  }

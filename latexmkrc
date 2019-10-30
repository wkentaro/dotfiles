# vim: ft=perl
my $os = `uname`;

if ( $os =~ /linux/i ) {
  $pdf_previewer = 'start okular';
} else {
  $pdf_previewer = "open -a /Applications/Skim.app";
}

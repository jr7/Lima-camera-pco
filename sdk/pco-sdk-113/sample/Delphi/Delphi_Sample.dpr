program Delphi_Sample;

uses
  Forms,
  Delphi_Test in 'Delphi_Test.pas' {Form1};

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TForm1, Form1);
  Application.Run;
end.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace GroupedNativeMethodsGenerator;

[Generator(LanguageNames.CSharp)]
public partial class GroupedNativeMethodsGenerator : IIncrementalGenerator
{
    public void Initialize(IncrementalGeneratorInitializationContext context)
    {
        context.RegisterPostInitializationOutput(ctx =>
        {
            ctx.AddSource("GroupedNativeMethodsGenerator.Attribute.cs", """
namespace GroupedNativeMethodsGenerator
{
    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
    internal sealed class GroupedNativeMethodsAttribute : Attribute
    {
        public string RemovePrefix { get; set; } = "";
        public bool RemoveUntilTypeName { get; set; } = true;

        /// <summary>Fix method name as C# Style, e.g.: foo_bar_baz() -> FooBarBaz()</summary>
        public bool FixMethodName { get; set; } = true;
    }
}
""");
        });

        var source = context.SyntaxProvider.ForAttributeWithMetadataName("GroupedNativeMethodsGenerator.GroupedNativeMethodsAttribute",
            (node, token) => node is ClassDeclarationSyntax,
            (ctx, token) => ctx);

        context.RegisterSourceOutput(source, Emit);
    }

    static void Emit(SourceProductionContext context, GeneratorAttributeSyntaxContext source)
    {
        var typeSymbol = (INamedTypeSymbol)source.TargetSymbol;
        var typeNode = (TypeDeclarationSyntax)source.TargetNode;

        var ns = typeSymbol.ContainingNamespace.IsGlobalNamespace
            ? ""
            : $"namespace {typeSymbol.ContainingNamespace};";

        var fullType = typeSymbol.ToDisplayString(SymbolDisplayFormat.FullyQualifiedFormat)
            .Replace("global::", "")
            .Replace("<", "_")
            .Replace(">", "_");

        var methods = typeSymbol.GetMembers()
            .OfType<IMethodSymbol>();



        // context.AddSource($"{fullType}.SampleGenerator.g.cs", code);
    }
}

// TODO:...
public static class DiagnosticDescriptors
{
    const string Category = "SampleGenerator";

    public static readonly DiagnosticDescriptor ExistsOverrideToString = new(
        id: "SAMPLE001",
        title: "ToString override",
        messageFormat: "The GenerateToString class '{0}' has ToString override but it is not allowed.",
        category: Category,
        defaultSeverity: DiagnosticSeverity.Error,
        isEnabledByDefault: true);
}